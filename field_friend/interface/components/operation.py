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
        self.plant_locator = system.plant_locator
        self.plant_provider = system.plant_provider
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
                    if self.plant_locator is not None:
                        with ui.expansion('Plant Provider').classes('w-full').bind_value(app.storage.user, 'show_plant_provider_settings'), ui.row().classes('items-center'):
                            self.plant_provider.settings_ui()
                    with ui.expansion('Detections').classes('w-full').bind_value(app.storage.user, 'show_detection_settings'), ui.row().classes('items-center'):
                        self.plant_locator.settings_ui()

        with activities:
            self.navigation_selection = ui.select(
                list(self.system.navigation_strategies.keys()),
                on_change=self.handle_navigation_changed,
                label='Navigation'
            ).classes('w-32') \
                .tooltip('Select the navigation strategy') \
                .bind_value_from(self.system, 'current_navigation', lambda i: i.name)
            assert self.system.current_navigation is not None
            self.navigation_selection.value = self.system.current_navigation.name

            self.implement_selection = ui.select(
                list(self.system.implements.keys()),
                on_change=self.handle_implement_changed,
                label='Implement') \
                .classes('w-32') \
                .tooltip('Select the implement to work with') \
                .bind_value_from(self.system, 'current_implement', lambda i: i.name)
            assert self.system.current_implement is not None
            self.implement_selection.value = self.system.current_implement.name

    def handle_implement_changed(self, e: events.ValueChangeEventArguments) -> None:
        assert self.system.current_implement is not None
        if self.system.current_implement.name != e.value:
            self.system.current_implement = self.system.implements[e.value]
        self.implement_settings.clear()
        assert self.system.current_implement is not None
        with self.implement_settings:
            self.system.current_implement.settings_ui()

    def handle_navigation_changed(self, e: events.ValueChangeEventArguments) -> None:
        assert self.system.current_navigation is not None
        if self.system.current_navigation.name != e.value:
            self.system.current_navigation = self.system.navigation_strategies[e.value]
        self.navigation_settings.clear()
        assert self.system.current_navigation is not None
        with self.navigation_settings:
            self.system.current_navigation.settings_ui()

    def edit_selected_field(self, parameters: dict):
        if self.field_provider.selected_field:
            name = self.field_provider.selected_field.name
            self.field_provider.update_field_parameters(
                field_id=self.field_provider.selected_field.id,
                name=parameters['name'],
                row_count=int(parameters['row_count']),
                row_spacing=float(parameters['row_spacing']),
                outline_buffer_width=float(parameters['outline_buffer_width']),
                bed_count=int(parameters['bed_count']),
                bed_spacing=float(parameters['bed_spacing']),
                bed_crops=parameters['bed_crops']
            )
            ui.notify(f'Parameters of Field "{name}" has been changed')
        else:
            ui.notify('No field selected', color='warning')
        if self.edit_field_dialog:
            self.edit_field_dialog.close()

    def _delete_selected_field(self):
        if self.field_provider.selected_field:
            name = self.field_provider.selected_field.name
            self.field_provider.delete_selected_field()
            ui.notify(f'Field "{name}" has been deleted')
        else:
            ui.notify('No field selected', color='warning')
        if self.delete_field_dialog:
            self.delete_field_dialog.close()

    # TODO: move to field_creator or something
    @ui.refreshable
    def field_setting(self) -> None:
        with ui.dialog() as self.edit_field_dialog, ui.card():
            parameters: dict = {
                'name': self.field_provider.selected_field.name if self.field_provider.selected_field else '',
                'row_count': self.field_provider.selected_field.row_count if self.field_provider.selected_field else 0,
                'row_spacing': self.field_provider.selected_field.row_spacing if self.field_provider.selected_field else 0.0,
                'outline_buffer_width': self.field_provider.selected_field.outline_buffer_width if self.field_provider.selected_field else 2.0,
                'bed_count': self.field_provider.selected_field.bed_count if self.field_provider.selected_field else 1,
                'bed_spacing': self.field_provider.selected_field.bed_spacing if self.field_provider.selected_field else 0.5,
                'bed_crops': self.field_provider.selected_field.bed_crops if self.field_provider.selected_field else {'0': None}
            }
            with ui.tabs().classes('w-full') as tabs:
                one = ui.tab('General')
                two = ui.tab('Beds')
            with ui.tab_panels(tabs, value=two).classes('w-full'):
                with ui.tab_panel(one):
                    ui.input('Field Name', value=parameters['name']) \
                        .props('dense outlined').classes('w-full') \
                        .bind_value(parameters, 'name')
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
                        .bind_value(parameters, 'row_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0) \
                        .tooltip('Set the spacing between rows')
                    ui.number('Outline Buffer Width', value=parameters.get(
                        'outline_buffer_width', 2.0), min=1, max=10, step=0.1, suffix='m') \
                        .props('dense outlined').classes('w-full') \
                        .bind_value(parameters, 'outline_buffer_width') \
                        .tooltip('Set the outline buffer width')
                with ui.tab_panel(two):
                    ui.number('Number of Beds', value=parameters['bed_count'], min=1, step=1) \
                        .props('dense outlined').classes('w-full') \
                        .bind_value(parameters, 'bed_count') \
                        .tooltip('Set the number of beds')
                    ui.separator()
                    if self.plant_locator is not None:
                        with ui.column().classes('w-full').style('max-height: 500px; overflow-y: auto;'):
                            for i in range(parameters['bed_count']):
                                with ui.row().classes('w-full item-center'):
                                    ui.label(f'Bed {i + 1}:').classes('text-lg')
                                    ui.select(self.plant_locator.crop_category_names) \
                                        .props('dense outlined').classes('w-40') \
                                        .bind_value(parameters, 'bed_crops',
                                                    forward=lambda v, idx=i: {
                                                        **parameters.get('bed_crops', {}), str(idx): v},
                                                    backward=lambda v, idx=i: v.get(str(idx)))
            with ui.row():
                ui.button('Cancel', on_click=self.edit_field_dialog.close)
                ui.button('Apply', on_click=lambda: self.edit_selected_field(parameters))

        with ui.dialog() as self.delete_field_dialog, ui.card():
            ui.label('Are you sure you want to delete this field?')
            with ui.row():
                ui.button('Cancel', on_click=self.delete_field_dialog.close)
                ui.button('Delete', on_click=self._delete_selected_field, color='red')
        with ui.row().style('width:100%;'):
            ui.button(icon='add_box', text='Field', on_click=lambda: FieldCreator(self.system)) \
                .tooltip('Create a field with AB-line in a few simple steps')
        if len(self.field_provider.fields) <= 0:
            return
        with ui.row().classes('w-full mt-2'):
            self.field_select = ui.select(
                value=self.field_provider.selected_field.id if self.field_provider.selected_field else None,
                options={field.id: field.name for field in self.field_provider.fields},
                label='Select Field',
                on_change=lambda e: self.field_provider.select_field(e.value)
            ).classes('w-3/4')
            if self.field_provider.selected_field:
                with ui.row():
                    ui.button(icon='edit', on_click=self.edit_field_dialog.open) \
                        .classes('ml-2').tooltip('Edit the selected field')
                    ui.button(icon='add_box', text='Row Point', on_click=lambda: SupportPointDialog(self.system)) \
                        .tooltip('Add a support point for a row')
                    ui.button(icon='delete', on_click=self.delete_field_dialog.open) \
                        .props('color=red').classes('ml-2').tooltip('Delete the selected field')
                if self.field_provider.selected_field.bed_count > 1:
                    with ui.row().classes('w-full'):
                        beds_checkbox = ui.checkbox('Select specific beds').classes('w-full') \
                            .bind_value(self.system.field_provider, 'only_specific_beds')
                        with ui.row().bind_visibility_from(beds_checkbox, 'value').classes('w-full'):
                            ui.select(list(range(1, int(self.field_provider.selected_field.bed_count) + 1)),
                                      multiple=True, label='selected beds', clearable=True) \
                                .classes('grow').props('use-chips') \
                                .bind_value(self.field_provider, 'selected_beds')
