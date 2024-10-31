from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point, Pose

from ..field import Field, Row
from ..implements import Implement
from .follow_crops_navigation import FollowCropsNavigation
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from system import System


class State(Enum):
    APPROACHING_ROW_START = auto()
    FOLLOWING_ROW = auto()
    ROW_COMPLETED = auto()
    FIELD_COMPLETED = auto()


class FieldNavigation(StraightLineNavigation):
    DRIVE_STEP = 0.2
    TURN_STEP = np.deg2rad(25.0)
    MAX_GNSS_WAITING_TIME = 15.0

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self._state = State.APPROACHING_ROW_START
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

    @property
    def current_row(self) -> Row:
        assert self.field
        return self.field.rows[self.row_index]

    async def prepare(self) -> bool:
        await super().prepare()
        self.field = self.field_provider.get_field(self.field_id)
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return False
        if not self.field.rows:
            rosys.notify('No rows available', 'negative')
            return False
        if self.gnss.device is None:
            rosys.notify('GNSS is not available', 'negative')
            return False
        for idx, row in enumerate(self.field.rows):
            if not len(row.points) >= 2:
                rosys.notify(f'Row {idx} on field {self.field.name} has not enough points', 'negative')
                return False
        self.row_index = self.field.rows.index(self.get_nearest_row())
        self._state = State.APPROACHING_ROW_START
        self.plant_provider.clear()

        self.automation_watcher.start_field_watch(self.field.outline)

        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    def _should_finish(self) -> bool:
        return self._state == State.FIELD_COMPLETED

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.stop_field_watch()
        await self.implement.deactivate()

    def get_nearest_row(self) -> Row:
        assert self.field is not None
        assert self.gnss.device is not None
        row = min(self.field.rows, key=lambda r: r.line_segment().line.foot_point(
            self.odometer.prediction.point).distance(self.odometer.prediction.point))
        self.log.info(f'Nearest row is {row.name}')
        self.row_index = self.field.rows.index(row)
        return row

    def set_start_and_end_points(self):
        assert self.field is not None
        self.start_point = None
        self.end_point = None
        relative_point_0 = self.odometer.prediction.distance(self.current_row.points[0].cartesian())
        relative_point_1 = self.odometer.prediction.distance(self.current_row.points[-1].cartesian())
        # self.log.info(f'Relative point 0: {relative_point_0} Relative point 1: {relative_point_1}')
        if relative_point_0 < relative_point_1:
            self.start_point = self.current_row.points[0].cartesian()
            self.end_point = self.current_row.points[-1].cartesian()
        elif relative_point_1 < relative_point_0:
            self.start_point = self.current_row.points[-1].cartesian()
            self.end_point = self.current_row.points[0].cartesian()

        relative_start = self.odometer.prediction.relative_point(self.start_point).x
        relative_end = self.odometer.prediction.relative_point(self.end_point).x
        if relative_start < 0 <= relative_end:
            self.start_point, self.end_point = self.end_point, self.start_point
        self.update_target()
        # self.log.info(f'Start point: {self.start_point} End point: {self.end_point}')

    def update_target(self) -> None:
        self.origin = self.odometer.prediction.point
        if self.end_point is None:
            return
        self.target = self.end_point

    async def _drive(self, distance: float) -> None:
        assert self.field is not None
        # TODO: remove temporary fix for GNSS waiting
        if self.odometer.prediction.distance(self.gnss._last_gnss_pose) > 1.0:  # pylint: disable=protected-access
            await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)

        if self._state == State.APPROACHING_ROW_START:
            self._state = await self._run_approaching_row_start()
        elif self._state == State.FOLLOWING_ROW:
            self._state = await self._run_following_row(distance)
        elif self._state == State.ROW_COMPLETED:
            self._state = await self._run_row_completed()

    async def _run_approaching_row_start(self) -> State:
        self.set_start_and_end_points()
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()

        await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        # turn towards row start
        target_yaw = self.odometer.prediction.direction(self.start_point)
        await self.turn_in_steps(target_yaw)

        # TODO: IF INSIDE FIELD AREA:
        # distance_to_row = self.current_row.line_segment().line.foot_point(
        #     self.odometer.prediction.point).distance(self.odometer.prediction.point)
        # if distance_to_row < 0.1:
        #     rosys.notify('Inside field area', 'positive')
        #     return State.FOLLOWING_ROW
        # else:
        #     rosys.notify('Between rows', 'negative')
        #     return State.FIELD_COMPLETED
        # END TODO

        # drive to row start
        await self.drive_in_steps(Pose(x=self.start_point.x, y=self.start_point.y, yaw=target_yaw))
        await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        # turn to row
        assert self.start_point is not None
        assert self.end_point is not None
        row_yaw = self.start_point.direction(self.end_point)
        await self.turn_in_steps(row_yaw)
        return State.FOLLOWING_ROW

    async def drive_in_steps(self, target: Pose) -> None:
        while True:
            if target.relative_point(self.odometer.prediction.point).x > 0:
                return
            drive_step = min(self._drive_step, self.odometer.prediction.distance(target))
            # Calculate timeout based on linear speed limit and drive step
            timeout = (drive_step / self.driver.parameters.linear_speed_limit) + 3.0
            await self._drive_towards_target(drive_step, target, timeout=timeout)
            await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)

    async def turn_to_yaw(self, target_yaw, angle_threshold=np.deg2rad(1.0)) -> None:
        while True:
            angle = rosys.helpers.eliminate_2pi(target_yaw - self.odometer.prediction.yaw)
            if abs(angle) < angle_threshold:
                break
            linear = 0.5
            sign = 1 if angle > 0 else -1
            angular = linear / self.driver.parameters.minimum_turning_radius * sign
            await self.driver.wheels.drive(*self.driver._throttle(linear, angular))  # pylint: disable=protected-access
            await rosys.sleep(0.1)
        await self.driver.wheels.stop()

    async def turn_in_steps(self, target_yaw: float) -> None:
        angle_difference = rosys.helpers.angle(self.odometer.prediction.yaw, target_yaw)
        while abs(angle_difference) > np.deg2rad(1.0):
            next_angle = rosys.helpers.eliminate_2pi(
                self.odometer.prediction.yaw + np.sign(angle_difference) * self._turn_step)
            if abs(angle_difference) > self._turn_step:
                await self.turn_to_yaw(next_angle)
            else:
                await self.turn_to_yaw(target_yaw)
            await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
            angle_difference = rosys.helpers.angle(self.odometer.prediction.yaw, target_yaw)

    async def _run_following_row(self, distance: float) -> State:
        end_pose = rosys.geometry.Pose(x=self.end_point.x, y=self.end_point.y,
                                       yaw=self.start_point.direction(self.end_point), time=0)
        if end_pose.relative_point(self.odometer.prediction.point).x > 0:
            await self.driver.wheels.stop()
            await self.implement.deactivate()
            return State.ROW_COMPLETED
        if not self.implement.is_active:
            await self.implement.activate()
        self.update_target()
        await super()._drive(distance)
        return State.FOLLOWING_ROW

    async def _run_row_completed(self) -> State:
        await self.driver.wheels.stop()
        assert self.field
        next_state: State = State.ROW_COMPLETED
        if not self._loop and self.current_row == self.field.rows[-1]:
            return State.FIELD_COMPLETED
        self.row_index += 1
        next_state = State.APPROACHING_ROW_START

        # TODO: rework later, when starting at any row is possible
        if self.row_index >= len(self.field.rows):
            if self._loop:
                self.row_index = 0
            else:
                next_state = State.FIELD_COMPLETED
        return next_state

    def backup(self) -> dict:
        return super().backup() | {
            'field_id': self.field.id if self.field else None,
            'row_index': self.row_index,
            'state': self._state.name,
            'loop': self._loop,
            'drive_step': self._drive_step,
            'turn_step': self._turn_step,
            'max_gnss_waiting_time': self._max_gnss_waiting_time,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self.row_index = data.get('row_index', 0)
        self._state = State[data.get('state', State.APPROACHING_ROW_START.name)]
        self._loop = data.get('loop', False)
        self._drive_step = data.get('drive_step', self.DRIVE_STEP)
        self._turn_step = data.get('turn_step', self.TURN_STEP)
        self._max_gnss_waiting_time = data.get('max_gnss_waiting_time', self.MAX_GNSS_WAITING_TIME)

    def settings_ui(self) -> None:
        with ui.row():
            super().settings_ui()

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')
        ui.number('Drive Step', step=0.01, min=0.01, max=10.0, format='%.2f', suffix='m', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, '_drive_step') \
            .tooltip(f'DRIVE_STEP (default: {self.DRIVE_STEP:.2f})')
        ui.number('Turn Step', step=1.0, min=1.0, max=180.0, format='%.1f', suffix='°', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, '_turn_step', forward=np.deg2rad, backward=np.rad2deg) \
            .tooltip(f'TURN_STEP (default: {np.rad2deg(self.TURN_STEP):.2f})')
        ui.number('Max GNSS Waiting Time', step=0.1, min=0.1, format='%.1f', suffix='s', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, '_max_gnss_waiting_time') \
            .tooltip(f'MAX_GNSS_WAITING_TIME (default: {self.MAX_GNSS_WAITING_TIME:.2f})')

    def _set_field_id(self) -> None:
        self.field_id = self.field_provider.selected_field.id if self.field_provider.selected_field else None

    def create_simulation(self, crop_distance: float = 0.3) -> None:
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
                        .polar(randint(-15, 15)*0.01, self.odometer.prediction.yaw + np.pi/2)
                    self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                        position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
