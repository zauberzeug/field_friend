
import logging
from typing import TYPE_CHECKING

from nicegui import events, ui

from .key_controls import KeyControls
from .leaflet_map import leaflet_map
from .punch_dialog import PunchDialog

if TYPE_CHECKING:
    from field_friend.system import System


class operation:

    def __init__(self, system: 'System', map: leaflet_map) -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field_provider = system.field_provider
        self.field = None
        self.key_controls = KeyControls(self.system)
        self.leaflet_map = map

        with ui.row().classes('w-full').style('min-height: 100%; width: 55%;'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    activities = ui.row().classes('items-center')
                    with ui.row():
                        ui.label('Settings').classes('text-xl')
                    self.implement_settings = ui.column()

        with activities:
            self.implement_selection = ui.select(
                [key for key in self.system.implements.keys()],
                on_change=self.handle_implement_changed,
                label='Implement') \
                .classes('w-32') \
                .tooltip('Select the implement to work with') \
                .bind_value_from(self.system, 'current_implement', lambda i: i.name)
            self.implement_selection.value = self.system.current_implement.name

            self.field_selection = ui.select(
                {f.id: f.name for f in self.system.field_provider.fields},
                on_change=self.set_field,
                label='Field')\
                .props('clearable') \
                .classes('w-32') \
                .tooltip('Select the field to work on')
            # NOTE: having this in a separate call will trigger the on_change handler which is necessary to perform all the required updates (eg. self.set_field)
            self.field_selection \
                .bind_value_from(self.field_provider, 'active_field', lambda f: f.id if f else None)

        self.system.puncher.POSSIBLE_PUNCH.register(self.can_punch)
        self.punch_dialog = PunchDialog(self.system.usb_camera_provider, self.system.plant_locator)

    def set_field(self) -> None:
        if self.field_selection.value is None:
            self.field_provider.select_field()
            return
        for field in self.system.field_provider.fields:
            if field.id == self.field_selection.value:
                self.field_provider.select_field(field)
                if len(field.points) > 0:
                    self.system.gnss.reference = field.points[0]
                # TODO das hier noch auf das active field umbauen, damit auch diese werte im weeding auf das active field registriert sind
                # self.system.weeding.field = field
                # self.system.mowing.field = field
                # self.show_start_row.refresh()
                # self.show_end_row.refresh()

    async def can_punch(self, plant_id: str) -> None:
        self.punch_dialog.label.text = 'Do you want to punch at the current position?'
        self.punch_dialog.target_plant = self.system.plant_provider.get_plant_by_id(plant_id)
        result = await self.punch_dialog
        if result == 'Yes':
            self.system.puncher.punch_allowed = 'allowed'
        elif result == 'No':
            self.system.puncher.punch_allowed = 'not_allowed'
        elif result == 'Cancel':
            self.system.puncher.punch_allowed = 'not_allowed'

    def handle_implement_changed(self, e: events.ValueChangeEventArguments) -> None:
        if self.system.current_implement.name != e.value:
            self.system.current_implement = self.system.implements[e.value]
        self.implement_settings.clear()
        with self.implement_settings:
            self.system.current_implement.settings_ui()
