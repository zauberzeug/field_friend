import gc
from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import Point, Point3d, Pose, Spline

from ..field import Field, Row
from ..implements.implement import Implement
from ..implements.weeding_implement import WeedingImplement
from .navigation import WorkflowException, is_reference_valid
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from ...system import System


class State(Enum):
    APPROACH_START_ROW = auto()
    CHANGE_ROW = auto()
    FOLLOW_ROW = auto()
    WAITING_FOR_CONFIRMATION = auto()
    ROW_COMPLETED = auto()
    FIELD_COMPLETED = auto()
    ERROR = auto()


class FieldNavigation(StraightLineNavigation):
    MAX_DISTANCE_DEVIATION = 0.05
    MAX_ANGLE_DEVIATION = np.deg2rad(10.0)
    THREE_POINT_TURN_RADIUS = 1.5

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
        self.three_point_turn_radius: float = self.THREE_POINT_TURN_RADIUS
        self.force_first_row_start: bool = False
        self.is_in_swarm: bool = False
        self.allowed_to_turn: bool = False
        self.wait_distance: float = 1.3

        self.field: Field | None = None
        self.field_id: str | None = self.field_provider.selected_field.id if self.field_provider.selected_field else None
        self.field_provider.FIELD_SELECTED.register(self._set_field_id)
        self._loop: bool = False
        self.rows_to_work_on: list[Row] = []

    @property
    def target_heading(self) -> float:
        assert self.field is not None
        assert self.start_point is not None
        assert self.end_point is not None
        return self.start_point.direction(self.end_point)

    @property
    def current_row(self) -> Row | None:
        assert self.field
        if len(self.rows_to_work_on) == 0:
            self.log.warning('No rows to work on')
            return None
        return self.rows_to_work_on[self.row_index]

    @track
    async def start(self) -> None:
        if not is_reference_valid(self.gnss):
            raise WorkflowException('reference to far away from robot')
        await super().start()

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
        if self.gnss is None or not self.gnss.is_connected:
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
        self.automation_watcher.gnss_watch_active = True
        self.automation_watcher.start_field_watch(self.field.outline)
        return True

    def _should_finish(self) -> bool:
        return self._state in (State.FIELD_COMPLETED, State.ERROR)

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.gnss_watch_active = False
        self.automation_watcher.stop_field_watch()
        await self.implement.deactivate()

    def get_nearest_row(self) -> Row | None:
        assert self.field is not None
        assert self.gnss is not None
        assert self.gnss.is_connected
        if self.force_first_row_start:
            self.row_index = 0
        else:
            row = min(self.rows_to_work_on, key=lambda r: r.line_segment().line.foot_point(
                self.robot_locator.pose.point).distance(self.robot_locator.pose.point))
            self.log.debug(f'Nearest row is {row.name}')
            if row not in self.rows_to_work_on:
                rosys.notify('Please place the robot in front of a selected bed\'s row', 'negative')
                return None
            self.row_index = self.rows_to_work_on.index(row)
        return self.rows_to_work_on[self.row_index]

    def set_start_and_end_points(self):
        assert self.field is not None
        self.start_point = None
        self.end_point = None
        if self.current_row is None:
            self.log.warning('No current row')
            return
        start_point = self.current_row.points[0].to_local()
        end_point = self.current_row.points[-1].to_local()
        swap_points: bool
        if self._is_in_working_area(start_point, end_point):
            abs_angle_to_start = abs(self.robot_locator.pose.relative_direction(start_point))
            abs_angle_to_end = abs(self.robot_locator.pose.relative_direction(end_point))
            swap_points = abs_angle_to_start < abs_angle_to_end
        else:
            distance_to_start = self.robot_locator.pose.distance(start_point)
            distance_to_end = self.robot_locator.pose.distance(end_point)
            swap_points = distance_to_start > distance_to_end
        if swap_points:
            self.log.debug('Swapping start and end points')
            start_point, end_point = end_point, start_point
        self.start_point = start_point
        self.end_point = end_point
        self.log.debug('Start point: %s End point: %s', self.start_point, self.end_point)
        self.update_target()

    def update_target(self) -> None:
        self.origin = self.robot_locator.pose.point
        if self.end_point is None:
            return
        self.target = self.end_point

    @track
    async def _drive(self) -> None:
        assert self.field is not None
        while not self._should_finish():
            if self._state == State.FOLLOW_ROW:
                self._state = await self._run_follow_row()
                continue
            with self.implement.blocked():
                if self._state == State.APPROACH_START_ROW:
                    self._state = await self._run_approach_start_row()
                elif self._state == State.CHANGE_ROW:
                    self._state = await self._run_change_row()
                elif self._state == State.ROW_COMPLETED:
                    self._state = await self._run_row_completed()
                elif self._state == State.WAITING_FOR_CONFIRMATION:
                    self._state = await self._run_waiting_for_confirmation()

    @track
    async def _run_approach_start_row(self) -> State:
        self.allowed_to_turn = False
        self.set_start_and_end_points()
        if self.start_point is None or self.end_point is None:
            return State.ERROR
        robot_in_working_area = self._is_in_working_area(self.start_point, self.end_point, position_error_margin=0.0)
        if not self._is_start_allowed(self.start_point, self.end_point, robot_in_working_area):
            return State.ERROR
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()

        if not robot_in_working_area:
            assert self.start_point is not None
            target_yaw = self.robot_locator.pose.direction(self.start_point)
            await self.turn_to_yaw(target_yaw)
            await self.driver.drive_to(self.start_point, backward=False)
            assert self.end_point is not None
        driving_yaw = self.robot_locator.pose.direction(self.end_point)
        await self.turn_to_yaw(driving_yaw)
        self._set_cultivated_crop()
        return State.FOLLOW_ROW

    @track
    async def _run_change_row(self) -> State:
        self.set_start_and_end_points()
        await self._run_three_point_turn(self.three_point_turn_radius)
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()
        self._set_cultivated_crop()
        self.allowed_to_turn = False
        return State.FOLLOW_ROW

    @track
    async def _run_three_point_turn(self, radius: float = 1.5) -> None:
        # TODO: tests fail with radius=1.5, but 1.499 and 1501 work perfectly fine
        assert self.start_point is not None
        assert self.end_point is not None
        t0_pose = self.robot_locator.pose
        direction_to_start = t0_pose.relative_direction(self.start_point)
        distance_to_start = t0_pose.distance(self.start_point)
        y_offset = max(radius, distance_to_start)
        t1_pose = t0_pose.transform_pose(
            Pose(x=radius, y=y_offset * np.sign(direction_to_start), yaw=direction_to_start))
        await self.driver.drive_spline(Spline.from_poses(t0_pose, t1_pose))
        row_yaw = self.start_point.direction(self.end_point)
        start_pose = Pose(x=self.start_point.x, y=self.start_point.y, yaw=row_yaw)
        t2_pose = start_pose.transform_pose(
            Pose(x=-radius, y=radius * np.sign(direction_to_start), yaw=-direction_to_start))
        with self.driver.parameters.set(can_drive_backwards=True):
            await self.driver.drive_to(target=t2_pose.point, backward=True)
        await self.driver.drive_spline(Spline.from_poses(t2_pose, start_pose))

    @track
    async def _run_follow_row(self) -> State:
        assert self.end_point is not None
        assert self.start_point is not None
        end_yaw = self.start_point.direction(self.end_point)
        end_pose = Pose(x=self.end_point.x, y=self.end_point.y, yaw=end_yaw)
        distance_from_end = end_pose.relative_point(self.robot_locator.pose.point).x
        if distance_from_end > 0:
            await self.driver.wheels.stop()
            await self.implement.deactivate()
            return State.ROW_COMPLETED
        if self.is_in_swarm and not self.allowed_to_turn and -self.wait_distance < distance_from_end < 0:
            await self.driver.wheels.stop()
            return State.WAITING_FOR_CONFIRMATION
        if not self.implement.is_active:
            gc.collect()
            await self.implement.activate()
        self.update_target()
        await self.drive_towards_target(end_pose)
        return State.FOLLOW_ROW

    @track
    async def _run_waiting_for_confirmation(self) -> State:
        await self.driver.wheels.stop()
        await rosys.sleep(0.1)
        if self.allowed_to_turn:
            return State.FOLLOW_ROW
        return State.WAITING_FOR_CONFIRMATION

    @track
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
            self.log.debug('Implement is not a weeding implement. Cannot set cultivated crop')
            return
        if self.current_row is None:
            self.log.warning('No current row')
            return
        if self.implement.cultivated_crop == self.current_row.crop:
            return
        self.implement.cultivated_crop = self.current_row.crop
        self.implement.request_backup()

    def _is_in_working_area(self, start_point: Point, end_point: Point, *, position_error_margin: float = 0.05) -> bool:
        # TODO: check if in working rectangle, current just checks if between start and stop
        relative_start = self.robot_locator.pose.relative_point(start_point)
        relative_end = self.robot_locator.pose.relative_point(end_point)
        robot_in_working_area = relative_start.x * relative_end.x <= 0.0
        if abs(relative_start.x) < position_error_margin or abs(relative_end.x) < position_error_margin:
            return False
        return robot_in_working_area

    def _is_start_allowed(self, start_point: Point, end_point: Point, robot_in_working_area: bool) -> bool:
        if not robot_in_working_area:
            self.log.debug('Robot is not in working area, will approach start row')
            return True
        if self.current_row is None:
            self.log.warning('No current row')
            return False
        foot_point = self.current_row.line_segment().line.foot_point(self.robot_locator.pose.point)
        distance_to_row = foot_point.distance(self.robot_locator.pose.point)
        if distance_to_row > self.MAX_DISTANCE_DEVIATION:
            rosys.notify('Robot is between two rows', 'negative')
            return False
        abs_angle_to_start = abs(self.robot_locator.pose.relative_direction(start_point))
        abs_angle_to_end = abs(self.robot_locator.pose.relative_direction(end_point))
        if abs_angle_to_start > self.MAX_ANGLE_DEVIATION and abs_angle_to_end > self.MAX_ANGLE_DEVIATION:
            rosys.notify('Robot heading deviates too much from row direction', 'negative')
            return False
        return True

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'field_id': self.field.id if self.field else None,
            'loop': self._loop,
            'three_point_turn_radius': self.three_point_turn_radius,
            'wait_distance': self.wait_distance,
            'force_first_row_start': self.force_first_row_start,
            'is_in_swarm': self.is_in_swarm,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self._loop = data.get('loop', False)
        self.three_point_turn_radius = data.get('three_point_turn_radius', self.three_point_turn_radius)
        self.force_first_row_start = data.get('force_first_row_start', self.force_first_row_start)
        self.wait_distance = data.get('wait_distance', self.wait_distance)
        self.is_in_swarm = data.get('is_in_swarm', self.is_in_swarm)

    def settings_ui(self) -> None:
        with ui.row():
            super().settings_ui()
            ui.number('Turn radius', step=0.1, min=0.0, max=10.0, format='%.1f', suffix='m', on_change=self.request_backup) \
                .props('dense outlined') \
                .classes('w-24') \
                .bind_value(self, 'three_point_turn_radius') \
                .tooltip(f'Radius when changing rows (default: {self.THREE_POINT_TURN_RADIUS:.1f}m)')

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('Field Navigation').classes('text-center text-bold')
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')
        ui.checkbox('Force first row start', on_change=self.request_backup).bind_value(self, 'force_first_row_start')
        ui.checkbox('Is in swarm', on_change=self.request_backup).bind_value(self, 'is_in_swarm')
        ui.checkbox('Allowed to turn').bind_value(self, 'allowed_to_turn')
        ui.number('Wait distance', step=0.1, min=0.0, max=10.0, format='%.1f', suffix='m', on_change=self.request_backup) \
            .bind_value(self, 'wait_distance').classes('w-20')

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
            assert self.current_row is not None
            crop = self.current_row.crop or 'maize'
            for i in range(int(crop_count)):
                p = self.start_point.interpolate(self.end_point, (crop_distance * (i+1)) / length)
                if i == 10:
                    p.y += 0.20
                else:
                    p.y += randint(-5, 5) * 0.01
                p3d = Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name=crop, position=p3d)
                self.detector.simulated_objects.append(plant)

                for _ in range(1, 7):
                    p = self.start_point.polar(crop_distance * (i+1) + randint(-5, 5) * 0.01, self.start_point.direction(self.end_point)) \
                        .polar(randint(-15, 15)*0.01, self.robot_locator.pose.yaw + np.pi/2)
                    self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                        position=Point3d(x=p.x, y=p.y, z=0)))
