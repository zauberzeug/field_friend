
import asyncio
import logging
from typing import TYPE_CHECKING

from nicegui import app, events, ui

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
                    with ui.expansion('Navigation').classes('w-full').bind_value(app.storage.user, 'show_navigation_settings'):
                        self.navigation_settings = ui.row().classes('items-center')
                    with ui.expansion('Implement').classes('w-full').bind_value(app.storage.user, 'show_implement_settings'):
                        self.implement_settings = ui.row().classes('items-center')
                    with ui.expansion('Plant Provider').classes('w-full').bind_value(app.storage.user, 'show_plant_provider_settings'), ui.row():
                        self.system.plant_provider.settings_ui()

        with activities:
            self.navigation_selection = ui.select(
                [key for key in self.system.navigation_strategies.keys()],
                on_change=self.handle_navigation_changed,
                label='Navigation') \
                .classes('w-32') \
                .tooltip('Select the navigation strategy') \
                .bind_value_from(self.system, 'current_navigation', lambda i: i.name)
            self.navigation_selection.value = self.system.current_navigation.name

            self.implement_selection = ui.select(
                [key for key in self.system.implements.keys()],
                on_change=self.handle_implement_changed,
                label='Implement') \
                .classes('w-32') \
                .tooltip('Select the implement to work with') \
                .bind_value_from(self.system, 'current_implement', lambda i: i.name)
            self.implement_selection.value = self.system.current_implement.name

        self.system.puncher.POSSIBLE_PUNCH.register(self.can_punch)
        self.punch_dialog = PunchDialog(self.system.usb_camera_provider,
                                        self.system.plant_locator,
                                        self.system.odometer)

    async def can_punch(self, plant_id: str) -> None:
        self.punch_dialog.target_plant = self.system.plant_provider.get_plant_by_id(plant_id)
        result: str | None = None
        try:
            result = await asyncio.wait_for(self.punch_dialog, timeout=self.punch_dialog.timeout)
        except asyncio.TimeoutError:
            self.punch_dialog.close()
            result = None
        if result == 'Yes':
            self.system.puncher.punch_allowed = 'allowed'
        elif result is None or result == 'No' or result == 'Cancel':
            self.system.puncher.punch_allowed = 'not_allowed'

    def handle_implement_changed(self, e: events.ValueChangeEventArguments) -> None:
        if self.system.current_implement.name != e.value:
            self.system.current_implement = self.system.implements[e.value]
        self.implement_settings.clear()
        with self.implement_settings:
            self.system.current_implement.settings_ui()

    def handle_navigation_changed(self, e: events.ValueChangeEventArguments) -> None:
        if self.system.current_navigation.name != e.value:
            self.system.current_navigation = self.system.navigation_strategies[e.value]
        self.navigation_settings.clear()
        with self.navigation_settings:
            self.system.current_navigation.settings_ui()
