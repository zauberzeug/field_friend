from enum import Enum, auto
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point

from ..field import Field, Row
from ..implements import Implement, WeedingImplement
from .follow_crops_navigation import FollowCropsNavigation
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from system import System


class State(Enum):
    APPROACHING_ROW_START = auto()
    FOLLOWING_ROW = auto()
    ROW_COMPLETED = auto()
    CLEAR_ROW = auto()
    FIELD_COMPLETED = auto()


class FieldNavigation(FollowCropsNavigation):
    CLEAR_ROW_DISTANCE = 0.5

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
        self.clear_row_distance: float = self.CLEAR_ROW_DISTANCE
        self._loop: bool = False

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
        # row = self.get_nearest_row()
        # self.row_index = self.field.rows.index(row)
        self.row_index = 0
        # if self._state == State.FIELD_COMPLETED:
        #     self.clear()
        # else:
        self._state = State.APPROACHING_ROW_START
        self.plant_provider.clear()

        await rosys.sleep(1)  # wait for GNSS to update
        # self.automation_watcher.start_field_watch(self.field.outline)

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
        # currently not used, starts on row 0
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
        elif self._state == State.CLEAR_ROW:
            self._state = await self._run_clear_row()
        elif self._state == State.ROW_COMPLETED:
            self._state = await self._run_row_completed()

    async def _run_approaching_row_start(self) -> State:
        # TODO only drive to row if we are not on any rows and near the row start
        self.set_start_and_end_points()
        await self._wait_for_gnss()
        await self._drive_to_row()
        await self._wait_for_gnss()
        # self.log.info(f'Following "{self.current_row.name}"...')
        self.plant_provider.clear()
        self.set_start_and_end_points()
        self.update_target()
        return State.FOLLOWING_ROW

    async def _run_following_row(self, distance: float) -> State:
        if not self.implement.is_active:
            await self.implement.activate()
        if StraightLineNavigation._should_finish(self):  # type: disable=W0212
            await self.implement.deactivate()
            return State.CLEAR_ROW
        await super()._drive(distance)
        return State.FOLLOWING_ROW

    async def _run_clear_row(self) -> State:
        target = self.odometer.prediction.transform_pose(rosys.geometry.Pose(x=self.clear_row_distance, y=0))
        await self._drive_towards_target(0.02, target)
        await self._wait_for_gnss()
        return State.ROW_COMPLETED

    async def _run_row_completed(self) -> State:
        assert self.field
        next_state: State = State.ROW_COMPLETED
        if not self._loop and self.current_row == self.field.rows[-1]:
            return State.FIELD_COMPLETED
        self.row_index += 1
        next_state = State.APPROACHING_ROW_START

        # TODO: remove later, when any direction is possible
        if self.row_index >= len(self.field.rows):
            if self._loop:
                self.row_index = 0
            else:
                next_state = State.FIELD_COMPLETED
        return next_state

    async def _wait_for_gnss(self, waiting_time: float = 1.0) -> None:
        self.gnss.is_paused = False
        await rosys.sleep(waiting_time)
        await self.gnss.update_robot_pose()
        self.gnss.is_paused = True

    async def _drive_to_row(self) -> None:
        # self.log.info(f'Driving to "{self.current_row.name}"...')
        assert self.field
        assert self.start_point
        assert self.end_point
        direction = self.start_point.direction(self.end_point)
        row_start_pose = Pose(x=self.start_point.x, y=self.start_point.y, yaw=direction)
        target_pose = row_start_pose.transform_pose(Pose(x=-self.clear_row_distance))
        # self.log.info(f'Driving to {end_pose} from {self.odometer.prediction}...')
        spline = Spline.from_poses(self.odometer.prediction, target_pose)
        # self.log.info(f'Driving spline {spline}...')
        await self.driver.drive_spline(spline)
        # self.log.info(f'Arrived at row {self.current_row.name} starting at {target}')

    def backup(self) -> dict:
        return super().backup() | {
            'field_id': self.field.id if self.field else None,
            'row_index': self.row_index,
            'state': self._state.name,
            'clear_row_distance': self.clear_row_distance,
            'loop': self._loop,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self.row_index = data.get('row_index', 0)
        self._state = State[data.get('state', State.APPROACHING_ROW_START.name)]
        self.clear_row_distance = data.get('clear_row_distance', self.CLEAR_ROW_DISTANCE)
        self._loop = data.get('loop', False)

    def clear(self) -> None:
        return
        self._state = State.APPROACHING_ROW_START
        self.row_index = 0

    def settings_ui(self) -> None:
        super().settings_ui()
        field_selection = ui.select(
            {f.id: f.name for f in self.field_provider.fields if len(f.rows) >= 1},
            on_change=lambda args: self._set_field(args.value),
            label='Field')\
            .classes('w-32') \
            .tooltip('Select the field to work on')
        field_selection.bind_value_from(self, 'field', lambda f: f.id if f else None)
        ui.number('Clear Row Distance', step=0.05, min=0.01, max=5.0, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'clear_row_distance') \
            .tooltip(f'Safety distance to row in m (default: {self.CLEAR_ROW_DISTANCE:.2f})')

    def developer_ui(self) -> None:
        # super().developer_ui()
        ui.label('').bind_text_from(self, '_state', lambda state: f'State: {state.name}')
        ui.label('').bind_text_from(self, 'row_index', lambda row_index: f'Row Index: {row_index}')
        ui.label('').bind_text_from(self, 'start_point', lambda start_point: f'Start Point: {start_point}')
        ui.label('').bind_text_from(self, 'end_point', lambda end_point: f'End Point: {end_point}')
        ui.checkbox('Loop', on_change=self.request_backup).bind_value(self, '_loop')

    def _set_field(self, field_id: str) -> None:
        field = self.field_provider.get_field(field_id)
        if field is not None:
            self.field = field
            # if isinstance(self.implement, WeedingImplement):
            #     self.implement.cultivated_crop = field.crop
            self.clear()
        else:
            if isinstance(self.implement, WeedingImplement):
                self.implement.cultivated_crop = None

    def create_simulation(self, crop_distance: float = 0.5) -> None:
        self.detector.simulated_objects.clear()
        if self.field is None:
            return
        for row in self.field.rows:
            if len(row.points) < 2:
                continue
            cartesian = row.cartesian()
            start = cartesian[0]
            end = cartesian[-1]
            length = start.distance(end)
            # self.log.info(f'Adding plants from {start} to {end} (length {length:.1f} m)')
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
