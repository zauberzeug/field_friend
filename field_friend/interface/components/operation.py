import logging
from typing import TYPE_CHECKING

from nicegui import app, events, ui

from .key_controls import KeyControls

if TYPE_CHECKING:
    from field_friend.system import System


class operation:

    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field_provider = system.field_provider
        self.field = None
        self.key_controls = KeyControls(self.system)

        with ui.row().classes('w-full').style('min-height: 100%; width: 55%;'):
            self.system.robot_locator.ui()
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    activities = ui.row().classes('items-center')
                    with ui.row():
                        ui.label('Settings').classes('text-xl')
                    with ui.expansion('Navigation').classes('w-full').bind_value(app.storage.user, 'show_navigation_settings'):
                        self.navigation_settings = ui.row().classes('items-center')
                    with ui.expansion('Implement').classes('w-full').bind_value(app.storage.user, 'show_implement_settings'):
                        self.implement_settings = ui.row().classes('items-center')
                    with ui.expansion('Plant Provider').classes('w-full').bind_value(app.storage.user, 'show_plant_provider_settings'), ui.row().classes('items-center'):
                        self.system.plant_provider.settings_ui()
                    with ui.expansion('Detections').classes('w-full').bind_value(app.storage.user, 'show_detection_settings'), ui.row().classes('items-center'):
                        self.system.plant_locator.settings_ui()

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
