from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any, Generator

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point

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


class FieldNavigation(FollowCropsNavigation):
    TURN_STEP = np.deg2rad(25.0)
    MAX_GNSS_WAITING_TIME = 10.0

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self._state = State.APPROACHING_ROW_START
        self.row_index = 0
        self.start_point: Point | None = None
        self.end_point: Point | None = None

        self.field: Field | None = None
        self._loop: bool = False
        self._turn_step = self.TURN_STEP
        self._max_gnss_waiting_time = self.MAX_GNSS_WAITING_TIME

    @property
    def current_row(self) -> Row:
        assert self.field
        return self.field.rows[self.row_index]

    async def prepare(self) -> bool:
        await super().prepare()
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
        # TODO: allow starting from any row in a later PR
        # row = self.get_nearest_row()
        # self.row_index = self.field.rows.index(row)
        self.row_index = 0
        self._state = State.APPROACHING_ROW_START
        self.plant_provider.clear()

        self.automation_watcher.start_field_watch(self.field.outline)
        self.automation_watcher.gnss_watch_active = True

        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    def _should_finish(self) -> bool:
        return self._state == State.FIELD_COMPLETED

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.stop_field_watch()
        self.automation_watcher.gnss_watch_active = False
        await self.implement.deactivate()

    def get_nearest_row(self) -> Row:
        # TODO: currently not used, starts on row 0
        # assert self.field is not None
        # assert self.gnss.device is not None
        # row = min(self.field.rows, key=lambda r: r.line_segment().line.foot_point(
        #     self.odometer.prediction.point).distance(self.odometer.prediction.point))
        # self.log.info(f'Nearest row is {row.name}')
        # self.row_index = self.field.rows.index(row)
        # return row
        return Row(id='dummy', name='dummy', points=[])

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
        self.update_target()
        # self.log.info(f'Start point: {self.start_point} End point: {self.end_point}')

    def update_target(self) -> None:
        self.origin = self.odometer.prediction.point
        if self.end_point is None:
            return
        self.target = self.end_point

    async def _drive(self, distance: float) -> None:
        assert self.field is not None
        if self._state == State.APPROACHING_ROW_START:
            self._state = await self._run_approaching_row_start()
        elif self._state == State.FOLLOWING_ROW:
            self._state = await self._run_following_row(distance)
        elif self._state == State.ROW_COMPLETED:
            self._state = await self._run_row_completed()

    async def _run_approaching_row_start(self) -> State:
        def interpolate_angles(start: float, end: float, step: float) -> Generator[float, None, None]:
            total_diff = rosys.helpers.eliminate_2pi(end - start)
            num_steps = int(np.ceil(abs(total_diff) / step))
            for i in range(1, num_steps + 1):
                angle = rosys.helpers.eliminate_2pi(start + total_diff * i / num_steps)
                yield angle

        self.set_start_and_end_points()
        await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        target_yaw = self.odometer.prediction.direction(self.start_point)
        # turn towards row start
        for angle in interpolate_angles(self.odometer.prediction.yaw, target_yaw, self._turn_step):
            await self.turn_to_yaw(angle)
            await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        # drive to row start
        await self.driver.drive_to(self.start_point)
        await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)
        # adjust heading
        row_yaw = self.start_point.direction(self.end_point)
        for angle in interpolate_angles(self.odometer.prediction.yaw, row_yaw, self._turn_step):
            await self.turn_to_yaw(angle)
            await self.gnss.ROBOT_POSE_LOCATED.emitted(self._max_gnss_waiting_time)

        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.create_simulation()
        else:
            self.plant_provider.clear()
        return State.FOLLOWING_ROW

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

    async def _run_following_row(self, distance: float) -> State:
        if not self.implement.is_active:
            await self.implement.activate()
        if StraightLineNavigation._should_finish(self):  # pylint: disable=protected-access
            await self.implement.deactivate()
            return State.ROW_COMPLETED
        await super()._drive(distance)
        return State.FOLLOWING_ROW

    async def _run_row_completed(self) -> State:
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
        self._turn_step = data.get('turn_step', self.TURN_STEP)
        self._max_gnss_waiting_time = data.get('max_gnss_waiting_time', self.MAX_GNSS_WAITING_TIME)

    def settings_ui(self) -> None:
        with ui.row():
            super().settings_ui()
            field_selection = ui.select(
                {f.id: f.name for f in self.field_provider.fields if len(f.rows) >= 1},
                on_change=lambda args: self._set_field(args.value),
                label='Field')\
                .classes('w-32') \
                .tooltip('Select the field to work on')
            field_selection.bind_value_from(self, 'field', lambda f: f.id if f else None)

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')
        ui.number('Turn Step', step=1.0, min=1.0, max=180.0, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, '_turn_step', forward=np.deg2rad, backward=np.rad2deg)
        ui.number('Turn Step', step=0.1, min=0.1, max=20.0, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, '_max_gnss_waiting_time')

    def _set_field(self, field_id: str) -> None:
        field = self.field_provider.get_field(field_id)
        if field is not None:
            self.field = field
        self.field_provider.FIELDS_CHANGED.emit()

    def create_simulation(self, crop_distance: float = 0.5) -> None:
        self.detector.simulated_objects.clear()
        self.plant_provider.clear()
        if self.field is None:
            return
        if self.start_point is not None and self.end_point is not None:
            start = self.start_point
            end = self.end_point
            length = start.distance(end)
            crop_count = length / crop_distance
            for i in range(int(crop_count)):
                p = start.interpolate(end, (crop_distance * i) / length)
                p3d = rosys.geometry.Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name='maize', position=p3d)
                self.detector.simulated_objects.append(plant)

                for _ in range(1, 7):
                    p = start.polar(crop_distance * i + randint(-5, 5) * 0.01,
                                    start.direction(end)) \
                        .polar(randint(-15, 15)*0.01, self.odometer.prediction.yaw + np.pi/2)
                    self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                        position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
