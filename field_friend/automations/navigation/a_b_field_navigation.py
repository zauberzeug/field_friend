import math
from enum import Enum, auto
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.geometry import Point, Pose, Spline

from ..field import Field, Row
from ..implements import Implement, WeedingImplement
from .follow_crops_navigation import FollowCropsNavigation

if TYPE_CHECKING:
    from system import System


class ABFieldNavigatoin(FollowCropsNavigation):
    class State(Enum):
        APPROACHING_ROW_START = auto()
        FOLLOWING_ROW = auto()
        ROW_COMPLETED = auto()
        FIELD_COMPLETED = auto()

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)

        self.name = 'A B field navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self.state = self.State.APPROACHING_ROW_START
        self.field: Field | None = None
        self.row_index = 0
        self.start_point: Point | None = None
        self.end_point: Point | None = None

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
        self.current_row = self.get_nearest_row()
        self.row_index = self.field.rows.index(row)
        if self.state == self.State.FIELD_COMPLETED:
            self.clear()
        if self.state == self.State.FOLLOWING_ROW:
            if self.current_row.line_segment().distance(self.odometer.prediction.point) > 0.1:
                self.clear()
        else:
            self.plant_provider.clear()

        await rosys.sleep(1)  # wait for GNSS to update
        self.automation_watcher.start_field_watch(self.field.outline)

        # determine start and end point
        self.set_start_and_end_points()
        assert self.start_point is not None
        assert self.end_point is not None
        self.log.info(f'Start point: {self.start_point} End point: {self.end_point}')

        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    def _should_finish(self) -> bool:
        return self.state == self.State.FIELD_COMPLETED

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
        relative_point_0 = self.odometer.prediction.relative_point(self.current_row.points[0].cartesian())
        relative_point_1 = self.odometer.prediction.relative_point(self.current_row.points[-1].cartesian())
        if relative_point_0.x < 0 or relative_point_0.x < relative_point_1.x:
            self.start_point = self.current_row.points[0].cartesian()
            self.end_point = self.current_row.points[-1].cartesian()
        elif relative_point_1.x < 0 or relative_point_1.x < relative_point_0.x:
            self.start_point = self.current_row.points[-1].cartesian()
            self.end_point = self.current_row.points[0].cartesian()

    def update_target(self) -> None:
        pass

    async def _drive(self, distance: float) -> None:
        assert self.field is not None
        if self.state == self.State.APPROACHING_ROW_START:
            # TODO only drive to row if we are not on any rows and near the row start
            self.set_start_and_end_points()
            await self._drive_to_row()
            self.state = self.State.FOLLOWING_ROW
            self.log.info(f'Following "{self.current_row.name}"...')
            self.plant_provider.clear()
        if self.state == self.State.FOLLOWING_ROW:
            if not self.implement.is_active:
                await self.implement.activate()
            if self.odometer.prediction.point.distance(self.end_point) >= 0.1:
                await super()._drive(distance)
            else:  # TODO: drive forward to clear the row and not drive over the plants
                await self.implement.deactivate()
                self.state = self.State.ROW_COMPLETED
        if self.state == self.State.ROW_COMPLETED:
            if self.current_row == self.field.rows[-1]:
                self.state = self.State.FIELD_COMPLETED
                await rosys.sleep(0.1)  # wait for base class to finish navigation
            else:
                self.row_index += 1
                self.state = self.State.APPROACHING_ROW_START

    async def _drive_to_row(self):
        self.log.info(f'Driving to "{self.current_row.name}"...')
        assert self.field
        target = self.start_point
        direction = target.direction(self.end_point)
        end_pose = Pose(x=target.x, y=target.y, yaw=direction)
        spline = Spline.from_poses(self.odometer.prediction, end_pose)
        await self.driver.drive_spline(spline)
        self.log.info(f'Arrived at row {self.current_row.name} starting at {target}')

    def backup(self) -> dict:
        return super().backup() | {
            'field_id': self.field.id if self.field else None,
            'row_index': self.row_index,
            'state': self.state.name,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
        self.row_index = data.get('row_index', 0)
        self.state = self.State[data.get('state', self.State.APPROACHING_ROW_START.name)]

    def clear(self) -> None:
        self.state = self.State.APPROACHING_ROW_START
        self.row_index = 0

    def settings_ui(self) -> None:
        super().settings_ui()
        field_selection = ui.select(
            {f.id: f.name for f in self.field_provider.fields if len(f.rows) >= 1 and len(f.points) >= 3},
            on_change=lambda args: self._set_field(args.value),
            label='Field')\
            .classes('w-32') \
            .tooltip('Select the field to work on')
        field_selection.bind_value_from(self, 'field', lambda f: f.id if f else None)

    def _set_field(self, field_id: str) -> None:
        field = self.field_provider.get_field(field_id)
        if field is not None:
            self.field = field
            if isinstance(self.implement, WeedingImplement):
                self.implement.cultivated_crop = field.crop
            self.clear()
        else:
            if isinstance(self.implement, WeedingImplement):
                self.implement.cultivated_crop = None

    def create_simulation(self) -> None:
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
            self.log.info(f'Adding plants from {start} to {end} (length {length:.1f} m)')
            crop_count = length / 0.15
            for i in range(int(crop_count)):
                p = start.interpolate(end, (0.15 * i) / length)
                p3d = rosys.geometry.Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name='maize', position=p3d)
                self.detector.simulated_objects.append(plant)
