import math
from enum import Enum, auto
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.geometry import Point, Pose, Spline

from ..field import Field, Row
from ..implements.implement import Implement
from .follow_crops_navigation import FollowCropsNavigation

if TYPE_CHECKING:
    from system import System


class State(Enum):
    APPROACHING_ROW_START = auto()
    FOLLOWING_ROW = auto()
    RETURNING_TO_START = auto()
    ROW_COMPLETED = auto()
    FIELD_COMPLETED = auto()


class FieldNavigation(FollowCropsNavigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)

        self.name = 'Field Rows'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self.state = State.APPROACHING_ROW_START
        self.field: Field | None = None
        self.row_index = 0

    async def prepare(self) -> bool:
        if self.field is None:
            rosys.notify('No field selected', 'negative')
            return False
        if not self.field.reference:
            rosys.notify('No field reference available', 'negative')
            return False
        self.gnss.reference = self.field.reference
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

        self.automation_watcher.start_field_watch(self.field.outline)
        self.automation_watcher.gnss_watch_active = True
        return True

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.gnss_watch_active = False
        self.automation_watcher.stop_field_watch()

    async def _drive(self) -> None:
        assert self.field is not None
        assert self.field.reference is not None
        if self.state == State.APPROACHING_ROW_START:
            # TODO only drive to row if we are not on any rows and near the row start
            await self._drive_to_row(self.current_row)
            await self.implement.activate()
            self.state = State.FOLLOWING_ROW
        if self.state == State.FOLLOWING_ROW:
            if self.odometer.prediction.point.distance(self.current_row.points[-1].cartesian(self.field.reference)) >= 0.1:
                await super()._drive()
            else:
                await self.implement.deactivate()
                self.state = State.RETURNING_TO_START
        if self.state == State.RETURNING_TO_START:
            self.driver.parameters.can_drive_backwards = True
            end = self.current_row.points[0].cartesian(self.field.reference)
            await self.driver.drive_to(end, backward=True)  # TODO replace with following crops or replay recorded path
            inverse_yaw = (self.odometer.prediction.yaw + math.pi) % (2 * math.pi)
            await self.driver.drive_to(end.polar(1.5, inverse_yaw), backward=True)
            self.driver.parameters.can_drive_backwards = False
            self.state = State.ROW_COMPLETED
        if self.state == State.ROW_COMPLETED:
            if self.current_row == self.field.rows[-1]:
                self.state = State.FIELD_COMPLETED
                await rosys.sleep(0.1)  # wait for base class to finish navigation
            else:
                self.row_index += 1
                self.state = State.APPROACHING_ROW_START

    def _should_finish(self) -> bool:
        return self.state == State.FIELD_COMPLETED

    async def _drive_to_row(self, row: Row):
        self.log.info(f'Driving to row {row.name}...')
        assert self.field and self.field.reference
        target = row.points[0].cartesian(self.field.reference)
        direction = target.direction(row.points[-1].cartesian(self.field.reference))
        end_pose = Pose(x=target.x, y=target.y, yaw=direction)
        spline = Spline.from_poses(self.odometer.prediction, end_pose)
        await self.driver.drive_spline(spline)
        self.log.info(f'Arrived at row {row.name} starting at {target}')

    def backup(self) -> dict:
        return {
            'field_id': self.field.id if self.field else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)

    def settings_ui(self) -> None:
        field_selection = ui.select(
            {f.id: f.name for f in self.field_provider.fields},
            on_change=lambda args: self._set_field(args.value),
            label='Field')\
            .props('clearable') \
            .classes('w-32') \
            .tooltip('Select the field to work on')
        field_selection.bind_value_from(self, 'field', lambda f: f.id if f else None)
        super().settings_ui()

    def _set_field(self, field_id: str) -> None:
        field = self.field_provider.get_field(field_id)
        if field is not None:
            self.field = field
            if len(field.points) > 2 and len(field.rows) > 0:
                self.gnss.reference = field.points[0]
            else:
                rosys.notify(f'{field.name} is invalid', 'negative')

    def create_simulation(self):
        self.detector.simulated_objects.clear()
        if self.field is None:
            return
        for row in self.field.rows:
            if len(row.points) < 2:
                continue
            cartesian = row.cartesian(self.field.reference)
            start = cartesian[0]
            end = cartesian[-1]
            length = start.distance(end)
            self.log.info(f'Creating row from {start} to {end} with length {length}')
            crop_count = length / 0.15
            for i in range(int(crop_count)):
                p = start.interpolate(end, (0.15 * i) / length)
                p3d = rosys.geometry.Point3d(x=p.x, y=p.y, z=0)
                plant = rosys.vision.SimulatedObject(category_name='maize', position=p3d)
                self.detector.simulated_objects.append(plant)

    @property
    def current_row(self) -> Row:
        assert self.field
        return self.field.rows[self.row_index]
