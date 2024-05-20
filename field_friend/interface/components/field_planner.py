import logging
from typing import Literal

import rosys
from nicegui import events, ui

from ...automations import FieldProvider
from ...navigation import Gnss
from .geodata_picker import geodata_picker
from .leaflet_map import leaflet_map
from .operation import operation


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss, leaflet_map: leaflet_map) -> None:
        self.log = logging.getLogger('field_friend.field_planner')
        self.field_provider = field_provider
        self.operation = operation
        self.odometer = odometer
        self.gnss = gnss
        self.leaflet_map = leaflet_map
        self.field_provider.active_object = None
        self.coordinate_type = "WGS84"
        self.COORDINATE_TYPE_CHANGED = rosys.event.Event()
        "switch between displaying cartesian and wgs84 coordinates."

        self.tab: Literal["Obstacles", "Outline", "Rows"] = "Outline"
        self.TAB_CHANGED = rosys.event.Event()

        with ui.row().classes('w-full').style('height: 100%; max-height:100%; width: 100%;'):
            with ui.card().style('width: 48%; max-width: 48%; max-height: 100%; height: 100%;'):
                with ui.row():
                    # self.show_coordinate_type_selection()
                    ui.button('Upload Field', on_click=lambda field_provider=self.field_provider: geodata_picker(field_provider)) \
                        .tooltip('Upload a file with field boundaries. Supported file formats: KML, XML and Shape') \
                        .classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                    ui.button('Add field', on_click=self.field_provider.create_field).tooltip('Add a new field') \
                        .classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                    ui.button('Clear fields', on_click=self.field_provider.clear_fields).props('outline color=warning').tooltip('Delete all fields') \
                        .classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                with ui.row().style('width: 100%;'):
                    self.show_field_table()
                    self.field_provider.FIELDS_CHANGED.register(self.show_field_table.refresh)
            self.show_field_settings()
            self.field_provider.FIELD_SELECTED.register(self.show_field_settings.refresh)
            self.field_provider.FIELDS_CHANGED.register(self.show_field_settings.refresh)
            self.TAB_CHANGED.register(self.show_field_settings.refresh)
            self.COORDINATE_TYPE_CHANGED.register(self.show_field_settings.refresh)

            self.show_object_settings()
            self.field_provider.OBJECT_SELECTED.register(self.show_object_settings.refresh)
            self.COORDINATE_TYPE_CHANGED.register(self.show_object_settings.refresh)

    def set_tab(self, e: events.ValueChangeEventArguments) -> None:
        self.tab = e.value
        self.field_provider.select_object(None)
        self.TAB_CHANGED.emit()

    def handle_coordinate_type_change(self, e: events.ValueChangeEventArguments) -> None:
        self.coordinate_type = e.value
        self.COORDINATE_TYPE_CHANGED.emit()

    def table_selected(self, selection):
        if len(selection.selection) > 0:
            if self.field_provider.active_field is not None and (selection.selection[0]['id'] == self.field_provider.active_field.id):
                return
            for field in self.field_provider.fields:
                if field.id == selection.selection[0]['id']:
                    self.field_provider.select_field(field)
                    if len(field.points) > 0:
                        self.gnss.reference = field.points[0]
        else:
            self.field_provider.select_field(None)

    def show_coordinate_type_selection(self) -> None:
        ui.select(["WGS84", "cartesian"], value=self.coordinate_type, on_change=self.handle_coordinate_type_change)

    @ui.refreshable
    def show_field_table(self) -> None:
        columns = [
            {'name': 'id', 'label': 'id', 'field': 'id', 'required': True, 'align': 'center'},
            {'name': 'name', 'label': 'Name', 'field': 'name', 'required': True, 'align': 'left'},
            {'name': 'boundary', 'label': 'Boundary', 'field': 'boundary', 'sortable': True, 'align': 'center'},
            {'name': 'obstacles', 'label': 'Obstacles', 'field': 'obstacles', 'sortable': True, 'align': 'center'},
            {'name': 'rows', 'label': 'Rows', 'field': 'rows', 'sortable': True, 'align': 'center'},
        ]
        rows = []
        for field in self.field_provider.fields:
            data = {'id': field.id,
                    'name': field.name,
                    'boundary': f'{len(field.points)} points',
                    'obstacles': f'{len(field.obstacles)}',
                    'rows': f'{len(field.rows)}'}
            rows.append(data)
        self.field_table = ui.table(columns=columns, rows=rows, row_key='id',  selection='single',
                                    on_select=self.table_selected, pagination=4).style('width: 98%; max-width: 98%;')
        columns[0]['classes'] = 'hidden'
        columns[0]['headerClasses'] = 'hidden'
        self.field_table.update()
        if self.field_provider.active_field is not None:
            active_field_data = {'id': self.field_provider.active_field.id,
                                 'name': self.field_provider.active_field.name,
                                 'boundary': f'{len(self.field_provider.active_field.points)} points',
                                 'obstacles': f'{len(self.field_provider.active_field.obstacles)}',
                                 'rows': f'{len(self.field_provider.active_field.rows)}'}
            self.field_table.selected.append(active_field_data)

    @ui.refreshable
    def show_field_settings(self) -> None:
        with ui.card().style('width: 25%; max-height: 100%; height: 100%;'):
            if self.field_provider.active_field is None:
                with ui.column().style('display: block; margin: auto;'):
                    ui.icon('fence').props('size=lg color=primary').style('display: block; margin: auto;')
                    ui.label("select a field").style('display: block; margin: auto; color: #6E93D6;')
            else:
                with ui.row().style('width: 100%'):
                    ui.icon('fence').props('size=lg color=primary').style(
                        "display:block; margin-top:auto; margin-bottom: auto;")
                    ui.input(value=f'{self.field_provider.active_field.name}').on('blur', self.field_provider.invalidate).bind_value(
                        self.field_provider.active_field, 'name').classes('w-32')
                    ui.button(on_click=lambda field=self.field_provider.active_field: self.field_provider.delete_field(field)) \
                        .props('icon=delete color=warning fab-mini flat').classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;').tooltip('Delete field')
                with ui.tabs().style('width: 100%;') as self.tabs:
                    ui.tab('Outline', 'Outline')
                    ui.tab('Obstacles', 'Obstacles')
                    ui.tab('Rows', 'Rows')
                with ui.tab_panels(self.tabs, value=f'{self.tab}', on_change=self.set_tab).style('width: 100%;') as self.panels:
                    with ui.tab_panel('Outline').style('width: 100%;'):
                        if self.coordinate_type == "cartesian":
                            for point in self.field_provider.active_field.outline:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_field.outline[self.field_provider.active_field.outline.index(point)])) \
                                        .props('icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                                    ui.separator()
                        elif self.coordinate_type == "WGS84":
                            for geo_point in self.field_provider.active_field.points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(point.tuple)) \
                                        .props('icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number('latitude', value=geo_point.lat, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field: self.field_provider.add_field_point(field, point, [event.value, point.long])) \
                                        .classes('w-20')
                                    ui.number('longitude', value=geo_point.long, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field: self.field_provider.add_field_point(field, point, [point.lat, event.value])) \
                                        .classes('w-20')
                                    ui.button(on_click=lambda point=geo_point,
                                              field=self.field_provider.active_field: self.field_provider.add_field_point(field, point)) \
                                        .props('icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                                    ui.separator()
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                ui.button('', on_click=lambda field=self.field_provider.active_field: self.field_provider.add_field_point(field)) \
                                    .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                ui.button('', on_click=lambda field=self.field_provider.active_field: self.field_provider.remove_field_point(field)) \
                                    .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')

                    with ui.tab_panel('Obstacles'):
                        with ui.row().style("width: 100%;"):
                            radio_el = {}
                            for obstacle in self.field_provider.active_field.obstacles:
                                radio_el[obstacle.id] = obstacle.name
                            if self.field_provider.active_object is not None and self.field_provider.active_object['object'] is not None and self.tab == "Obstacles":
                                obstacle_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.field_provider.select_object(event.value, self.tab), value=self.field_provider.active_object['object'].id)
                            else:
                                obstacle_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.field_provider.select_object(event.value, self.tab))
                        with ui.row().classes('items-center mt-3').style("width: 100%"):
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.field_provider.create_obstacle(field)) \
                                .props('color=primary outline').style("width: 100%")
                    with ui.tab_panel('Rows'):
                        with ui.row().style("width: 100%;"):
                            radio_el = {}
                            for row in self.field_provider.active_field.rows:
                                radio_el[row.id] = row.name
                            if self.field_provider.active_object is not None and self.field_provider.active_object['object'] is not None and self.tab == "Rows":
                                row_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.field_provider.select_object(event.value, self.tab), value=self.field_provider.active_object['object'].id)
                            else:
                                row_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.field_provider.select_object(event.value, self.tab))

                        with ui.row().classes('items-center mt-3').style("width: 100%"):
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.field_provider.create_row(field)) \
                                .props('color=primary outline').style("width: 100%")

                        with ui.row().classes('items-center mt-3').style("width: 100%"):
                            ui.button("sort rows",
                                      on_click=lambda field=self.field_provider.active_field: self.field_provider.sort_rows(field))

    @ui.refreshable
    def show_object_settings(self) -> None:
        with ui.card().style('width: 24%; max-height: 100%; height: 100%;'):
            if self.field_provider.active_object is None or self.field_provider.active_object['object'] is None:
                with ui.column().style('display: block; margin: auto;'):
                    ui.icon('extension').props('size=lg color=primary').style('display: block; margin: auto;')
                    ui.label("select an object").style('display: block; margin: auto; color: #6E93D6;')
            else:
                if self.tab == "Obstacles":
                    with ui.row().style("width: 100%;"):
                        ui.icon('dangerous').props('size=sm color=primary').style(
                            "display:block; margin-top:auto; margin-bottom: auto;")
                        ui.input(
                            'Obstacle name', value=f'{self.field_provider.active_object["object"].name}').on('blur', self.field_provider.invalidate).bind_value(
                            self.field_provider.active_object['object'], 'name').classes('w-32')
                        ui.button(on_click=lambda field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.field_provider.remove_obstacle(field, obstacle)) \
                            .props('icon=delete color=warning fab-mini flat').classes('ml-auto') \
                            .style("display:block; margin-top:auto; margin-bottom: auto;") \
                            .tooltip('Delete obstacle')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        assert self.field_provider.active_field is not None
                        assert self.field_provider.active_field.reference is not None
                        if self.coordinate_type == "cartesian":
                            points = self.field_provider.active_object['object'] \
                                .cartesian(self.field_provider.active_field.reference)
                            for point in points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points(self.field_provider.active_field.reference).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for geo_point in self.field_provider.active_object['object'].points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].cartesian.index(point)])) \
                                        .props('icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number('latitude', value=geo_point.lat, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.field_provider.add_obstacle_point(field, obstacle, point, [event.value, point.long])) \
                                        .classes('w-20')
                                    ui.number('longitude', value=geo_point.long, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.field_provider.add_obstacle_point(field, obstacle, point, [point.lat, event.value])) \
                                        .classes('w-20')
                                    ui.button(on_click=lambda point=geo_point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.field_provider.add_obstacle_point(field, obstacle, point)) \
                                        .props('icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-8')
                                ui.button('', on_click=lambda field=self.field_provider.active_field,
                                          obstacle=self.field_provider.active_object['object']: self.field_provider.add_obstacle_point(field=field, obstacle=obstacle)) \
                                    .props('icon=add color=primary fab-mini flat')
                                ui.button('', on_click=lambda obstacle=self.field_provider.active_object['object']: self.field_provider.remove_obstacle_point(obstacle)) \
                                    .props('icon=remove color=warning fab-mini flat')
                elif self.tab == "Rows":
                    with ui.row().style("width: 100%"):
                        ui.icon('spa').props('size=sm color=primary').style(
                            "height: 100%; margin-top:auto; margin-bottom: auto;")
                        with ui.column():
                            ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.field_provider.move_row(self.field_provider.active_field, row)) .props(
                                'icon=expand_less color=primary fab-mini flat').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                            ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.field_provider.move_row(self.field_provider.active_field, row, next=True)) .props(
                                'icon=expand_more color=primary fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                        ui.input(value=self.field_provider.active_object['object'].name).on('blur', self.field_provider.invalidate).bind_value(
                            self.field_provider.active_object['object'], 'name').classes('w-32')
                        ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.field_provider.remove_row(self.field_provider.active_field, row)).props(
                            'icon=delete color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Delete Row')
                    with ui.row().style("width: 100%;"):
                        ui.label(f'Row crops: {len(self.field_provider.active_object["object"].crops)}') \
                            .props('color=primary').style("display:block; margin-top:auto; margin-bottom: auto;")
                        ui.button(on_click=self.field_provider.active_object['object'].clear_crops) \
                            .props('icon=sym_o_compost color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Clear all row crops')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        if self.coordinate_type == "cartesian":
                            points = self.field_provider.active_object['object'] \
                                .points(self.field_provider.active_field.reference)
                            for point in points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points(self.field_provider.active_field.reference).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for geo_point in self.field_provider.active_object['object'].points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=geo_point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number('latitude', value=geo_point.lat, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.field_provider.add_row_point(field, row, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=geo_point.long, format='%.6f', step=0.1,
                                        on_change=lambda event, point=geo_point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.field_provider.add_row_point(field, row, point, [point[0], event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=geo_point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.field_provider.add_row_point(field, row, point)) \
                                        .props('icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2').style('display: block; margin: auto;'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                ui.button('', on_click=lambda field=self.field_provider.active_field,
                                          row=self.field_provider.active_object['object']: self.field_provider.add_row_point(field, row)) \
                                    .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                ui.button('', on_click=lambda row=self.field_provider.active_object['object']: self.field_provider.remove_row_point(row)) \
                                    .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')
