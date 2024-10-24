import logging
from typing import TYPE_CHECKING
import rosys
from field_friend.automations import Field
from nicegui import app, events, ui
from .support_point_dialog import SupportPointDialog
from .field_creator import FieldCreator
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
        self.field_provider.FIELDS_CHANGED.register_ui(self.field_setting.refresh)
        self.system.field_provider.FIELD_SELECTED.register_ui(self.field_setting.refresh)

        with ui.row().classes('w-full').style('min-height: 100%; width: 55%;'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    activities = ui.row().classes('items-center')
                    with ui.row():
                        ui.label('Settings').classes('text-xl')
                    with ui.expansion('Fields').classes('w-full').bind_value(app.storage.user, 'show_fields_settings'):
                        self.field_setting()
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
                label='Navigation'
            ).classes('w-32') \
                .tooltip('Select the navigation strategy') \
                .bind_value_from(self.system, 'current_navigation', lambda i: i.name)
            self.navigation_selection.value = self.system.current_navigation.name

            self.implement_selection = ui.select(
                [key for key in self.system.implements.keys()],
                on_change=self.handle_implement_changed,
                label='Implement')\
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

    def edit_selected_field(self, parameters):
        if self.system.field_provider.selected_field:
            name = 'placeholder'
            ui.notify(f'Parameters of Field "{name}" has been changed')
            self.field_provider.FIELDS_CHANGED.emit()
        else:
            ui.notify('No field selected', color='warning')
        self.edit_field_dialog.close()

    def delete_selected_field(self):
        if self.system.field_provider.selected_field:
            name = self.system.field_provider.selected_field.name
            self.system.field_provider.delete_selected_field()
            ui.notify(f'Field "{name}" has been deleted')
            self.field_provider.FIELDS_CHANGED.emit()
        else:
            ui.notify('No field selected', color='warning')
        self.delete_field_dialog.close()

    @ui.refreshable
    def field_setting(self) -> None:
        with ui.dialog() as self.edit_field_dialog, ui.card():
            ui.label('Are you sure you want to delete this field?')
            parameters: dict = {
                'name': self.field_provider.selected_field.name if self.field_provider.selected_field else '',
                'row_number': self.field_provider.selected_field.row_number if self.field_provider.selected_field else 0,
                'row_spacing': self.field_provider.selected_field.row_spacing if self.field_provider.selected_field else 0.0
            }
            ui.input('Field Name', value=parameters['name']) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'name')
            ui.input('Row Number', value=parameters['row_number']) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'row_number')
            ui.input('Row Spacing', value=parameters['row_spacing']) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'row_spacing')
            with ui.row():
                ui.button('Cancel', on_click=self.edit_field_dialog.close)
                ui.button('Apply', on_click=lambda: self.system.field_provider.update_field_parameters(
                    self.system.field_provider.selected_field.id,
                    parameters['name'],
                    int(parameters['row_number']),
                    float(parameters['row_spacing'])
                )).props('color=primary')

        with ui.dialog() as self.delete_field_dialog, ui.card():
            ui.label('Are you sure you want to delete this field?')
            with ui.row():
                ui.button('Cancel', on_click=self.delete_field_dialog.close)
                ui.button('Delete', on_click=self.delete_selected_field).props('color=red')

        with ui.row().style('width:100%;'):
            ui.button(icon='add_box', text="Field", on_click=lambda: FieldCreator(self.system)).tooltip("Build a field with AB-line in a few simple steps") \
                .tooltip("Build a field with AB-line in a few simple steps. Currently only one field will be saved.")
        if len(self.system.field_provider.fields) > 0:
            with ui.row().classes('w-full mt-2'):
                self.field_select = ui.select(
                    value=self.system.field_provider.selected_field.id if self.system.field_provider.selected_field else None,
                    options={field.id: field.name for field in self.system.field_provider.fields},
                    label='Select Field',
                    on_change=lambda e: self.system.field_provider.select_field(e.value)
                ).classes('w-3/4')
                if self.system.field_provider.selected_field:
                    ui.button(icon='edit', on_click=self.edit_field_dialog.open) \
                        .classes('ml-2') \
                        .tooltip('Delete the selected field')
                    ui.button(icon='delete', on_click=self.delete_field_dialog.open) \
                        .props('color=red') \
                        .classes('ml-2') \
                        .tooltip('Delete the selected field')
            if self.system.field_provider.selected_field:
                with ui.row().style('width:100%;'):
                    ui.button(icon='add_box', text="Row Point", on_click=lambda: SupportPointDialog(self.system)).tooltip(
                        "Add a support point for a row")
