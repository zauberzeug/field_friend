from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from nicegui import app, events, ui

from .field_creator import FieldCreator
from .key_controls import KeyControls
from .support_point_dialog import SupportPointDialog

if TYPE_CHECKING:
    from ...system import System


class Operation:

    def __init__(self, system: System) -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field_provider = system.field_provider
        self.field = None
        self.key_controls = KeyControls(self.system)
        self.field_provider.FIELDS_CHANGED.register_ui(self.field_setting.refresh)
        self.field_provider.FIELD_SELECTED.register_ui(self.field_setting.refresh)
        self.selected_beds: set[int] = set()
        self.delete_field_dialog: ui.dialog | None = None
        self.edit_field_dialog: ui.dialog | None = None

        with ui.row().classes('w-full').style('min-height: 100%; width: 55%;'):
            with ui.row().classes('m-4').style('width: calc(100% - 2rem)'):
                with ui.column().classes('w-full'):
                    activities = ui.row().classes('items-center')
                    with ui.row():
                        ui.label('Settings').classes('text-xl')
                    with ui.expansion('Fields').classes('w-full').bind_value(app.storage.user, 'show_fields_settings'):
                        self.field_setting()  # type: ignore
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
                list(self.system.navigation_strategies.keys()),
                on_change=self.handle_navigation_changed,
                label='Navigation'
            ).classes('w-32') \
                .tooltip('Select the navigation strategy') \
                .bind_value_from(self.system, 'current_navigation', lambda i: i.name)
            self.navigation_selection.value = self.system.current_navigation.name

            self.implement_selection = ui.select(
                list(self.system.implements.keys()),
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

    def edit_selected_field(self, parameters: dict):
        if self.system.field_provider.selected_field:
            name = self.system.field_provider.selected_field.name
            self.system.field_provider.update_field_parameters(
                self.system.field_provider.selected_field.id,
                parameters['name'],
                int(parameters['row_count']),
                float(parameters['row_spacing']),
                float(parameters['outline_buffer_width']),
                int(parameters['bed_count']),
                float(parameters['bed_spacing'])
            )
            ui.notify(f'Parameters of Field "{name}" has been changed')
        else:
            ui.notify('No field selected', color='warning')
        if self.edit_field_dialog:
            self.edit_field_dialog.close()

    def delete_selected_field(self):
        if self.system.field_provider.selected_field:
            name = self.system.field_provider.selected_field.name
            self.system.field_provider.delete_selected_field()
            ui.notify(f'Field "{name}" has been deleted')
        else:
            ui.notify('No field selected', color='warning')
        if self.delete_field_dialog:
            self.delete_field_dialog.close()

    @ui.refreshable
    def field_setting(self) -> None:
        with ui.dialog() as self.edit_field_dialog, ui.card():
            parameters: dict = {
                'name': self.field_provider.selected_field.name if self.field_provider.selected_field else '',
                'row_count': self.field_provider.selected_field.row_count if self.field_provider.selected_field else 0,
                'row_spacing': self.field_provider.selected_field.row_spacing if self.field_provider.selected_field else 0.0,
                'outline_buffer_width': self.field_provider.selected_field.outline_buffer_width if self.field_provider.selected_field else 2.0,
                'bed_count': self.field_provider.selected_field.bed_count if self.field_provider.selected_field else 1,
                'bed_spacing': self.field_provider.selected_field.bed_spacing if self.field_provider.selected_field else 0.5
            }
            ui.input('Field Name', value=parameters['name']) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'name')
            ui.number('Number of Beds', value=parameters['bed_count'], min=1, step=1) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'bed_count')
            ui.number('Bed Spacing', value=parameters['bed_spacing'], suffix='cm', min=1, step=1) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'bed_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0) \
                .bind_visibility_from(parameters, 'bed_count', backward=lambda v: v is not None and v > 1)
            ui.input('Row Number (per Bed)', value=parameters['row_count']) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'row_count') \
                .tooltip('Set the number of rows per bed.')
            ui.number('Row Spacing', value=parameters['row_spacing'], suffix='cm', min=1, step=1) \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'row_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0)
            ui.number('Outline Buffer Width', value=parameters.get('outline_buffer_width', 2.0), min=1, max=10, step=0.1, suffix='m') \
                .props('dense outlined').classes('w-full') \
                .bind_value(parameters, 'outline_buffer_width') \
                .tooltip('Set the outline buffer width')
            with ui.row():
                ui.button('Cancel', on_click=self.edit_field_dialog.close)
                ui.button('Apply', on_click=lambda: self.edit_selected_field(parameters))

        with ui.dialog() as self.delete_field_dialog, ui.card():
            ui.label('Are you sure you want to delete this field?')
            with ui.row():
                ui.button('Cancel', on_click=self.delete_field_dialog.close)
                ui.button('Delete', on_click=self.delete_selected_field).props('color=red')

        with ui.row().style('width:100%;'):
            ui.button(icon='add_box', text='Field', on_click=lambda: FieldCreator(self.system)) \
                .tooltip('Create a field with AB-line in a few simple steps')
        if len(self.system.field_provider.fields) <= 0:
            return
        with ui.row().classes('w-full mt-2'):
            self.field_select = ui.select(
                value=self.system.field_provider.selected_field.id if self.system.field_provider.selected_field else None,
                options={field.id: field.name for field in self.system.field_provider.fields},
                label='Select Field',
                on_change=lambda e: self.system.field_provider.select_field(e.value)
            ).classes('w-3/4')
            if self.system.field_provider.selected_field:
                with ui.row():
                    ui.button(icon='edit', on_click=self.edit_field_dialog.open) \
                        .classes('ml-2') \
                        .tooltip('Edit the selected field')
                    ui.button(icon='add_box', text='Row Point', on_click=lambda: SupportPointDialog(self.system)) \
                        .tooltip('Add a support point for a row')
                    ui.button(icon='delete', on_click=self.delete_field_dialog.open) \
                        .props('color=red') \
                        .classes('ml-2') \
                        .tooltip('Delete the selected field')
                if self.system.field_provider.selected_field.bed_count > 1:
                    with ui.row().classes('w-full'):
                        beds_checkbox = ui.checkbox('Select specific beds').classes(
                            'w-full').on_value_change(self.system.field_provider.clear_selected_beds)
                        with ui.row().bind_visibility_from(beds_checkbox, 'value').classes('w-full'):
                            ui.select(list(range(1, int(self.system.field_provider.selected_field.bed_count) + 1)), multiple=True, label='selected beds', clearable=True) \
                                .classes('grow').props('use-chips').bind_value(self.system.field_provider, 'selected_beds')
