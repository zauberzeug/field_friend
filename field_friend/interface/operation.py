
import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import events, ui

from .automation_controls import automation_controls
from .field_friend_object import field_friend_object
from .field_object import field_object
from .key_controls import KeyControls
from .leaflet_map import leaflet_map
from .plant_object import plant_objects
from .visualizer_object import visualizer_object
from ..falling_detection import RuturnToSafety

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
        self.initial_value = None
        with ui.dialog() as self.dialog, ui.card():
            ui.label('Do you want to continue the old mowing automation?')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.dialog.submit('Yes'))
                ui.button('No', on_click=lambda: self.dialog.submit('No'))
                ui.button('Cancel', on_click=lambda: self.dialog.submit('Cancel'))

        with ui.card().tight().classes('w-full h-full'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    with ui.row().classes('items-center'):
                        @ui.refreshable
                        def center_map_button() -> None:
                            if self.field_provider.active_field is not None and len(self.field_provider.active_field.outline_wgs84) > 0:
                                ui.button(on_click=lambda: self.leaflet_map.m.set_center(self.field_provider.active_field.outline_wgs84[0])).props(
                                    'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                            else:
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                        center_map_button()
                        self.field_provider.FIELD_SELECTED.register(center_map_button.refresh)

                        field_selection_dict = {}
                        if self.field_provider.fields is not None and len(self.field_provider.fields) > 0:
                            for field in self.system.field_provider.fields:
                                field_selection_dict[field.id] = field.name
                            self.initial_value = None if self.field_provider.active_field is None else self.field_provider.active_field.id
                        self.field_selection = None

                        @ui.refreshable
                        def show_field_selection() -> None:
                            self.field_selection = ui.select(
                                field_selection_dict,
                                with_input=True, on_change=self.set_field, label='Field', value=self.initial_value).tooltip(
                                'Select the field to work on').classes('w-24')
                        show_field_selection()
                        self.field_provider.FIELDS_CHANGED.register(show_field_selection.refresh)
                    ui.separator()
                    with ui.row():
                        ui.label("Automation").classes('text-xl')
                    with ui.row().classes('w-full'):
                        self.automations_toggle = ui.select(
                            [key for key in self.system.automations.keys()],
                            value='weeding').bind_value(
                            self.system.automator, 'default_automation', forward=lambda key: self.system.automations[key],
                            backward=lambda automation: next(
                                key for key, value in self.system.automations.items() if value == automation)).classes('w-full border pl-2').style('border: 2px solid #6E93D6; border-radius: 5px; background-color: #EEF4FA')
                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('padding', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'padding').tooltip('Set the padding for the mowing automation')
                            ui.number('lane distance', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'lane_distance').tooltip('Set the lane distance for the system. automation')
                            ui.number('number of outer lanes', value=3, step=1, min=3, format='%.0f').props('dense outlined').classes(
                                'w-24').bind_value(system.mowing, 'number_of_outer_lanes').tooltip('Set the number of outer lanes for the mowing automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='demo_weeding'):
                        if system.field_friend.z_axis:
                            ui.number('Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.demo_weeding, 'drill_depth').tooltip('Set the drill depth for the weeding automation')
                            ui.label('press PLAY to start weeding with the set drill depth')
                        else:
                            ui.label('This Field Friend has no weeding tool available')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='weeding'):
                        with ui.row():
                            mode = ui.toggle(
                                ['Bohren', 'Hacken'],
                                value='Bohren').bind_value(
                                system.demo_weeding, 'mode').props('outline')
                            ui.number(
                                'Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.weeding, 'drill_depth').tooltip(
                                'Set the drill depth for the weeding automation').bind_visibility_from(
                                mode, 'value', value='Bohren')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='Continuous weeding'):
                        with ui.row():
                            ui.number(
                                'Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.weeding_new, 'drill_depth').tooltip(
                                'Set the drill depth for the weeding automation')

                    with ui.column().bind_visibility_from(self.automations_toggle, 'value', value='collecting'):
                        with ui.row():
                            ui.number(
                                'Drill angle', format='%.0f', value=100, step=1, min=1, max=180).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.coin_collecting, 'angle').tooltip(
                                'Set the drill depth for the weeding automation')
                            ui.checkbox('with drilling', value=True).bind_value(
                                self.system.coin_collecting, 'with_drilling')
            ui.space()
            with ui.row().style("margin: 1rem; width: calc(100% - 2rem);"):
                with ui.column():
                    ui.button('emergency stop', on_click=lambda: system.field_friend.estop.set_soft_estop(True)).props('color=red').classes(
                        'py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active', value=False)
                    ui.button('emergency reset', on_click=lambda: system.field_friend.estop.set_soft_estop(False)).props(
                        'color=red-700 outline').classes('py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop,
                                                                                                   'is_soft_estop_active', value=True)
                ui.button('self rescue', on_click=self.system.to_safety.rescue).props('color=orange').classes(
                    'py-3 px-6 text-lg').bind_visibility(system.falling_detection, 'has_stopped', value=True)
                ui.button('self rescue in progress').props('color=orange outline').classes(
                    'py-3 px-6 text-lg').bind_visibility(system.falling_detection, 'rescue_in_progress_val', value=True)
                ui.button('rescue attemt stopped, reset', on_click=system.falling_detection.reset_emergency).props('color=orange').classes(
                    'py-3 px-6 text-lg').bind_visibility(system.falling_detection, 'rescue_stop', value=True)
                ui.button('reset', on_click=system.falling_detection.reset).props('color=orange outline').classes(
                    'py-3 px-6 text-lg').bind_visibility(system.falling_detection, 'has_stopped', value=True)

                ui.space()
                with ui.row():
                    automation_controls(self.system.automator, can_start=self.ensure_start)

    def set_field(self) -> None:
        for field in self.system.field_provider.fields:
            if field.id == self.field_selection.value:
                self.field_provider.select_field(field)
                if len(field.outline_wgs84) > 0:
                    self.system.gnss.set_reference(field.outline_wgs84[0][0], field.outline_wgs84[0][1])
                # TODO das hier noch auf das active field umbauen, damit auch diese werte im weeding auf das active field registriert sind
                self.system.weeding.field = field
                self.system.weeding_new.field = field

    async def ensure_start(self) -> bool:
        self.log.info('Ensuring start of automation')
        if not self.automations_toggle.value == 'mowing' or self.system.mowing.current_path is None:
            return True
        result = await self.dialog
        if result == 'Yes':
            self.system.mowing.continue_mowing = True
        elif result == 'No':
            self.system.mowing.continue_mowing = False
        elif result == 'Cancel':
            return False
        return True
