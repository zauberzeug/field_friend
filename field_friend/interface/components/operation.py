
import logging
from typing import TYPE_CHECKING

from nicegui import events, ui

from .key_controls import KeyControls
from .leaflet_map import leaflet_map
from .punch_dialog import PunchDialog

if TYPE_CHECKING:
    from field_friend.system import System


class operation:

    def __init__(self, system: 'System', leaflet_map: leaflet_map) -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field_provider = system.field_provider
        self.field = None
        self.key_controls = KeyControls(self.system)
        self.leaflet_map = leaflet_map

        with ui.card().tight().classes('w-full').style('min-height: 100%; width: 55%;'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    with ui.row().classes('items-center'):
                        self.field_selection = ui.select(
                            {f.id: f.name for f in self.system.field_provider.fields},
                            with_input=True,
                            on_change=self.set_field,
                            label='Field')\
                            .props('clearable').classes('w-full') \
                            .tooltip('Select the field to work on')
                        # NOTE: having this in a separate call will trigger the on_change handler which is necessary to perform all the required updates (eg. self.set_field)
                        self.field_selection \
                            .bind_value_from(self.field_provider, 'active_field', lambda f: f.id if f else None)
                    ui.separator()
                    with ui.row():
                        ui.label('Automation').classes('text-xl')
                    with ui.row().classes('w-full'):
                        self.automations_toggle = ui.select([key for key in self.system.automations.keys()],
                                                            on_change=self.handle_automation_changed) \
                            .classes('w-full border pl-2').style('border: 2px solid #6E93D6; border-radius: 5px; background-color: #EEF4FA')
                        self.automations_toggle.value = self.system.get_current_automation_id()
                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('Padding', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m').classes('w-24') \
                                .bind_value(system.mowing, 'padding') \
                                .tooltip('Set the padding for the mowing automation')
                            ui.number('Lane distance', value=0.5, step=0.1, min=0.0, format='%.1f') \
                                .props('dense outlined suffix=m') \
                                .classes('w-24').bind_value(system.mowing,   'lane_distance') \
                                .tooltip('Set the lane distance for the system. automation')
                            ui.number('Number of outer lanes', value=3, step=1, min=3, format='%.0f') \
                                .props('dense outlined').classes('w-28') \
                                .bind_value(system.mowing, 'number_of_outer_lanes') \
                                .tooltip('Set the number of outer lanes for the mowing automation')
                            ui.number('Min. turning radius', format='%.2f', value=0.5, step=0.05, min=0.1, max=2.0) \
                                .props('dense outlined suffix=m') \
                                .classes('w-32') \
                                .bind_value(self.system.mowing, 'minimum_turning_radius') \
                                .tooltip('Set the turning radius for the mowing automation')

                    ui.separator()
                    self.automation_settings = ui.column()

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
                self.system.weeding.field = field
                self.system.mowing.field = field
                self.show_start_row.refresh()
                self.show_end_row.refresh()

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

    def handle_automation_changed(self, e: events.ValueChangeEventArguments) -> None:
        automation = self.system.automations[e.value]
        self.system.automator.default_automation = automation.start
        self.automation_settings.clear()
        with self.automation_settings:
            automation.ui()
        self.system.AUTOMATION_CHANGED.emit(e.value)
        self.system.request_backup()
