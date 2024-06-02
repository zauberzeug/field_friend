from typing import TYPE_CHECKING, Any, Optional

import numpy as np
import rosys
from nicegui import ui
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline

from ..field import Field, Row
from ..implements.implement import Implement
from ..sequence import find_sequence
from .follow_crops_navigation import FollowCropsNavigation

if TYPE_CHECKING:
    from ...system import System


class FieldNavigation(FollowCropsNavigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)

        self.PATH_PLANNED = rosys.event.Event()
        '''Event that is emitted when the path is planed. The event contains the path as a list of PathSegments.'''

        self.name = 'Field Rows'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

        self.field: Field | None = None
        self.row: Row | None = None

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
        if not self.row:
            self.row = self.field.rows[0]
        else:
            rosys.notify('Resume on current row is not yet implemented', 'negative')
            self.row = self.field.rows[0]
            # return False
        if not len(self.row.points) >= 2:
            rosys.notify('Row has not enough points', 'negative')
            return False

        # TODO only drive to row if we are not on any rows and near the row start
        await self._drive_to_row(self.row)

        # self.automation_watcher.start_field_watch(self.field.outline)
        # self.automation_watcher.gnss_watch_active = True
        return True

    async def _drive_forward(self) -> None:
        await super()._drive_forward()

    def _should_stop(self) -> bool:
        # TODO return True if we have worked all rows
        return super()._should_stop()

    async def _drive_to_row(self, row: Row):
        self.log.info(f'Driving to row {row.name}...')
        assert self.field and self.field.reference
        target = row.points[0].cartesian(self.field.reference)
        direction = target.direction(row.points[-1].cartesian(self.field.reference))
        end_pose = Pose(x=target.x, y=target.y, yaw=direction)
        spline = Spline.from_poses(self.odometer.prediction, end_pose)
        await self.driver.drive_spline(spline)
        self.log.info(f'Arrived at row {row.name} starting at {target}')

    def clear(self) -> None:
        super().clear()
        self.field = None
        self.start_row_id = None
        self.end_row_id = None
        self.sorted_weeding_rows = []
        self.weeding_plan = []
        self.turn_paths = []
        self.current_row = None
        self.current_segment = None
        self.row_segment_completed = False
        self.PATH_PLANNED.emit()

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
