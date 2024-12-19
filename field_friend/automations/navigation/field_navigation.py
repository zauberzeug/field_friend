from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point

from ..field import Field, Row
from ..implements.implement import Implement
from ..implements.weeding_implement import WeedingImplement
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from ...system import System


class State(Enum):
    APPROACH_START_ROW = auto()
    CHANGE_ROW = auto()
    FOLLOW_ROW = auto()
    ROW_COMPLETED = auto()
    FIELD_COMPLETED = auto()
    ERROR = auto()


class FieldNavigation(StraightLineNavigation):
    DRIVE_STEP = 0.2
    TURN_STEP = np.deg2rad(25.0)
    MAX_GNSS_WAITING_TIME = 15.0
    MAX_DISTANCE_DEVIATION = 0.05
    MAX_ANGLE_DEVIATION = np.deg2rad(10.0)

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self._state = State.APPROACH_START_ROW
        self.row_index = 0
        self.start_point: Point | None = None
        self.end_point: Point | None = None

        self.field: Field | None = None
        self.field_id: str | None = self.field_provider.selected_field.id if self.field_provider.selected_field else None
        self.field_provider.FIELD_SELECTED.register(self._set_field_id)
        self._loop: bool = False
        self._drive_step = self.DRIVE_STEP
        self._turn_step = self.TURN_STEP
        self._max_gnss_waiting_time = self.MAX_GNSS_WAITING_TIME
        self.rows_to_work_on: list[Row] = []
        self.robot_in_working_area = False

    @property
    def current_row(self) -> Row:
        assert self.field
        return self.rows_to_work_on[self.row_index]

    async def prepare(self) -> bool:
        await super().prepare()
        self.field = self.field_provider.get_field(self.field_id)
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return False
        self.rows_to_work_on = self.field_provider.get_rows_to_work_on()
        if not self.rows_to_work_on or len(self.rows_to_work_on) == 0:
            rosys.notify('No rows available', 'negative')
            return False
        if not self.gnss.is_connected:
            rosys.notify('GNSS is not available', 'negative')
            return False
        for idx, row in enumerate(self.rows_to_work_on):
            if not len(row.points) >= 2:
                rosys.notify(f'Row {idx} on field {self.field.name} has not enough points', 'negative')
                return False
        nearest_row = self.get_nearest_row()
        if nearest_row is None:
            return False
        self._state = State.APPROACH_START_ROW
        self.plant_provider.clear()
        self.automation_watcher.start_field_watch(self.field.outline)
        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    def _should_finish(self) -> bool:
        return self._state in (State.FIELD_COMPLETED, State.ERROR)

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.stop_field_watch()
        await self.implement.deactivate()

    def get_nearest_row(self) -> Row | None:
        assert self.field is not None
        assert self.gnss.is_connected
        row = min(self.field.rows, key=lambda r: r.line_segment().line.foot_point(
            self.robot_locator.pose.point).distance(self.robot_locator.pose.point))
        self.log.info(f'Nearest row is {row.name}')
        if row not in self.rows_to_work_on:
            rosys.notify('Please place the robot in front of a selected bed\'s row', 'negative')
            return None
        self.row_index = self.rows_to_work_on.index(row)
        return self.rows_to_work_on[self.row_index]

    def set_start_and_end_points(self):
        assert self.field is not None
        self.start_point = None
        self.end_point = None
        relative_point_0 = self.robot_locator.pose.distance(self.current_row.points[0].to_local())
        relative_point_1 = self.robot_locator.pose.distance(self.current_row.points[-1].to_local())
        # self.log.info(f'Relative point 0: {relative_point_0} Relative point 1: {relative_point_1}')
        if relative_point_0 < relative_point_1:
            self.start_point = self.current_row.points[0].to_local()
            self.end_point = self.current_row.points[-1].to_local()
        elif relative_point_1 < relative_point_0:
            self.start_point = self.current_row.points[-1].to_local()
            self.end_point = self.current_row.points[0].to_local()
        self.update_target()

    def update_target(self) -> None:
        self.origin = self.robot_locator.pose.point
        if self.end_point is None:
            return
        self.target = self.end_point

    async def _drive(self, distance: float) -> None:
        assert self.field is not None
        if self._state == State.APPROACH_START_ROW:
            self._state = await self._run_approach_start_row()
        elif self._state == State.CHANGE_ROW:
            self._state = await self._run_change_row()
        elif self._state == State.FOLLOW_ROW:
            self._state = await self._run_follow_row(distance)
        elif self._state == State.ROW_COMPLETED:
            self._state = await self._run_row_completed()

    async def _run_approach_start_row(self) -> State:
        self.robot_in_working_area = False
        rosys.notify(f'Approaching row {self.current_row.name}')
        self.set_start_and_end_points()
        if self.start_point is None or self.end_point is None:
            return State.ERROR
        if not self._is_start_allowed(self.start_point, self.end_point, self.robot_in_working_area):
            return State.ERROR

        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()

        # turn towards row start
        assert self.start_point is not None
        target_yaw = self.robot_locator.pose.direction(self.start_point)
        await self.turn_to_yaw(target_yaw)
        # drive to row start
        await self.driver.drive_to(self.start_point, backward=False)
        # turn to row
        assert self.end_point is not None
        driving_yaw = self.odometer.prediction.direction(self.end_point)
        await self.turn_in_steps(driving_yaw)
        self._set_cultivated_crop()
        return State.FOLLOW_ROW

    async def _run_change_row(self) -> State:
        self.robot_in_working_area = False
        self.set_start_and_end_points()
        await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        # turn towards row start
        assert self.start_point is not None
        target_yaw = self.robot_locator.pose.direction(self.start_point)
        await self.turn_to_yaw(target_yaw)
        # drive to row start
        await self.driver.drive_to(self.start_point, backward=False)
        # turn to row
        assert self.end_point is not None
        row_yaw = self.start_point.direction(self.end_point)
        await self.turn_to_yaw(row_yaw)

        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()
        return State.FOLLOWING_ROW

    # TODO: growing error because of the threshold
    async def turn_to_yaw(self, target_yaw: float, angle_threshold: float | None = None) -> None:
        if angle_threshold is None:
            angle_threshold = np.deg2rad(1.0)
        while True:
            angle = rosys.helpers.eliminate_2pi(target_yaw - self.robot_locator.pose.yaw)
            if abs(angle) < angle_threshold:
                break
            linear = 0.5
            sign = 1 if angle > 0 else -1
            angular = linear / self.driver.parameters.minimum_turning_radius * sign
            await self.driver.wheels.drive(*self.driver._throttle(linear, angular))  # pylint: disable=protected-access
            await rosys.sleep(0.1)
        await self.driver.wheels.stop()

    async def _run_following_row(self, distance: float) -> State:
        assert self.end_point is not None
        assert self.start_point is not None
        end_pose = rosys.geometry.Pose(x=self.end_point.x, y=self.end_point.y,
                                       yaw=self.start_point.direction(self.end_point), time=0)
        if end_pose.relative_point(self.robot_locator.pose.point).x > 0:
            await self.driver.wheels.stop()
            await self.implement.deactivate()
            return State.ROW_COMPLETED
        if not self.implement.is_active:
            await self.implement.activate()
        self.update_target()
        await super()._drive(distance)
        return State.FOLLOW_ROW

    async def _run_row_completed(self) -> State:
        await self.driver.wheels.stop()
        assert self.field
        next_state: State = State.ROW_COMPLETED
        if not self._loop and self.current_row == self.rows_to_work_on[-1]:
            return State.FIELD_COMPLETED
        self.row_index += 1
        next_state = State.CHANGE_ROW

        # TODO: rework later, when starting at any row is possible
        if self.row_index >= len(self.rows_to_work_on):
            if self._loop:
                self.row_index = 0
            else:
                next_state = State.FIELD_COMPLETED
        return next_state

    def _set_cultivated_crop(self) -> None:
        if not isinstance(self.implement, WeedingImplement):
            return
        if self.implement.cultivated_crop == self.current_row.crop:
            return
        rosys.notify(f'Setting crop {self.current_row.crop} for {self.implement.name}')
        self.implement.cultivated_crop = self.current_row.crop
        self.implement.request_backup()

    def _is_in_working_area(self, start_point: Point, end_point: Point) -> bool:
        # TODO: check if in working rectangle, current just checks if between start and stop
        relative_start = self.odometer.prediction.relative_point(start_point)
        relative_end = self.odometer.prediction.relative_point(end_point)
        robot_in_working_area = relative_start.x * relative_end.x <= 0
        self.log.debug('Robot in working area: %s', robot_in_working_area)
        return robot_in_working_area

    def _is_start_allowed(self, start_point: Point, end_point: Point, robot_in_working_area: bool) -> bool:
        if not robot_in_working_area:
            return True
        foot_point = self.current_row.line_segment().line.foot_point(self.odometer.prediction.point)
        distance_to_row = foot_point.distance(self.odometer.prediction.point)
        if distance_to_row > self.MAX_DISTANCE_DEVIATION:
            rosys.notify('Between two rows', 'negative')
            return False
        abs_angle_to_start = abs(self.odometer.prediction.relative_direction(start_point))
        abs_angle_to_end = abs(self.odometer.prediction.relative_direction(end_point))
        if abs_angle_to_start > self.MAX_ANGLE_DEVIATION and abs_angle_to_end > self.MAX_ANGLE_DEVIATION:
            rosys.notify('Robot heading deviates too much from row direction', 'negative')
            return False
        return True

    def backup(self) -> dict:
        return super().backup() | {
            'field_id': self.field.id if self.field else None,
            'loop': self._loop,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self._loop = data.get('loop', False)

    def settings_ui(self) -> None:
        with ui.row():
            super().settings_ui()

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('Field Navigation').classes('text-center text-bold')
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')

    def _set_field_id(self) -> None:
        self.field_id = self.field_provider.selected_field.id if self.field_provider.selected_field else None

    def create_simulation(self, crop_distance: float = 0.3) -> None:
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        self.detector.simulated_objects.clear()
        self.plant_provider.clear()
        if self.field is None:
            return
        if self.start_point is not None and self.end_point is not None:
            length = self.start_point.distance(self.end_point)
            crop_count = length / crop_distance
            for i in range(int(crop_count)):
                p = self.start_point.interpolate(self.end_point, (crop_distance * (i+1)) / length)
                if i == 10:
                    p.y += 0.20
                else:
                    p.y += randint(-5, 5) * 0.01
                p3d = rosys.geometry.Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name='maize', position=p3d)
                self.detector.simulated_objects.append(plant)

                for _ in range(1, 7):
                    p = self.start_point.polar(crop_distance * (i+1) + randint(-5, 5) * 0.01, self.start_point.direction(self.end_point)) \
                        .polar(randint(-15, 15)*0.01, self.robot_locator.pose.yaw + np.pi/2)
                    self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                        position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
