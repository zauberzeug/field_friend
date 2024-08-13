from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.geometry import Point

from ..field import Field, Row
from ..implements.implement import Implement
from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class ABLineNavigation(Navigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.MAX_STRETCH_DISTANCE: float = 2.0
        self.STOP_DISTANCE: float = 0.1
        self.start_position = self.odometer.prediction.point
        self.name = 'A-B line'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider
        assert self.gnss is not None
        assert self.bms is not None
        assert self.automation_watcher is not None
        assert self.field_provider is not None
        self.field: Field | None = None
        self.row: Row | None = None
        self.start_point: Point | None = None
        self.end_point: Point | None = None

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
        self.row = self.get_nearest_row()
        if self.row is None:
            rosys.notify('No row found', 'negative')
            return False
        if not len(self.row.points) >= 2:
            rosys.notify(f'Row {self.row.name} on field {self.field.name} has not enough points', 'negative')
            return False
        self.gnss.is_paused = False
        await rosys.sleep(3)  # wait for GNSS to update
        self.automation_watcher.start_field_watch(self.field.outline)

        # determine start and end point
        relative_point_0 = self.odometer.prediction.relative_point(self.row.points[0].cartesian())
        relative_point_1 = self.odometer.prediction.relative_point(self.row.points[-1].cartesian())
        self.log.info(f'{relative_point_0=} - {relative_point_1=}')
        self.start_point = None
        self.end_point = None
        if relative_point_0.x < 0 or relative_point_0.x < relative_point_1.x:
            self.start_point = self.row.points[0].cartesian()
            self.end_point = self.row.points[-1].cartesian()
        elif relative_point_1.x < 0 or relative_point_1.x < relative_point_0.x:
            self.start_point = self.row.points[-1].cartesian()
            self.end_point = self.row.points[0].cartesian()
        assert self.start_point is not None
        assert self.end_point is not None
        self.log.info(f'Start point: {self.start_point} End point: {self.end_point}')

        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    async def finish(self) -> None:
        await super().finish()
        self.automation_watcher.stop_field_watch()
        await self.implement.deactivate()

    async def _drive(self, distance: float):
        assert self.field is not None
        assert self.row is not None
        assert self.start_point is not None
        assert self.end_point is not None
        current_position = self.odometer.prediction.point
        direction = self.start_point.direction(self.end_point)
        self.log.info(f'line direction: {direction} robot yaw: {self.odometer.prediction.yaw}')
        distance = min(distance, current_position.distance(self.end_point))
        line = self.row.line_segment().line
        foot_point = line.foot_point(self.odometer.prediction.point)
        target = foot_point.polar(distance, direction)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, angular_speed_limit=self.angular_speed_limit):
            await self.driver.drive_to(target, throttle_at_end=False, stop_at_end=False)

    def _should_finish(self) -> bool:
        assert self.row is not None
        distance = self.odometer.prediction.point.distance(self.end_point)
        if distance < self.STOP_DISTANCE:
            self.log.info(f'Row {self.row.name} completed')
            return True
        if self.bms.is_below_percent(20):
            self.log.error('Battery is low')
            return True
        return False

    def get_nearest_row(self) -> Row:
        assert self.field is not None
        assert self.gnss.device is not None
        row = min(self.field.rows, key=lambda r: r.line_segment().line.foot_point(
            self.odometer.prediction.point).distance(self.odometer.prediction.point))
        self.log.info(f'Nearest row is {row.name}')
        return row

    def _set_field(self, field_id: str) -> None:
        field = self.field_provider.get_field(field_id)
        if field is not None:
            self.field = field

    def settings_ui(self) -> None:
        super().settings_ui()
        field_selection = ui.select(
            {f.id: f.name for f in self.field_provider.fields if len(f.rows) >= 1 and len(f.points) >= 3},
            on_change=lambda args: self._set_field(args.value),
            label='Field')\
            .classes('w-32') \
            .tooltip('Select the field to work on')
        field_selection.bind_value_from(self, 'field', lambda f: f.id if f else None)

    def backup(self) -> dict:
        return super().backup() | {
            'field_id': self.field.id if self.field else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        field_id = data.get('field_id', self.field_provider.fields[0].id if self.field_provider.fields else None)
        self.field = self.field_provider.get_field(field_id)
