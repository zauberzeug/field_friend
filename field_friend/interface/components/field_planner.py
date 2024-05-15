import logging
import uuid
from typing import Literal, Optional

import rosys
from nicegui import events, ui

from ...automations import Field, FieldObstacle, FieldProvider, Row
from ...navigation import GeoPoint, Gnss
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
        "switch between displaying cartesian and wgs84 coordiantes."

        self.tab: Literal["Obstacles", "Outline", "Rows"] = "Outline"
        self.TAB_CHANGED = rosys.event.Event()

        with ui.row().classes('w-full').style('height: 100%; max-height:100%; width: 100%;'):
            with ui.card().style('width: 48%; max-width: 48%; max-height: 100%; height: 100%;'):
                with ui.row():
                    self.show_coordinate_type_selection()
                    ui.button('Upload Field', on_click=lambda field_provider=self.field_provider: geodata_picker(field_provider)).tooltip(
                        'Upload a file with field boundaries. Supported file formates: KML, XML and Shape').classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                    ui.button('Add field', on_click=self.add_field).tooltip('Add a new field').classes(
                        'ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                    ui.button('Clear fields', on_click=self.clear_fields).props(
                        'outline color=warning').tooltip('Delete all fields').classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
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

    def set_tab(self, e: events.GenericEventArguments) -> None:
        self.tab = e.value
        self.field_provider.select_object(None)
        self.TAB_CHANGED.emit()

    def handle_coordinate_type_change(self, e: events.GenericEventArguments) -> None:
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
                        self.gnss.set_reference(field.points[0])
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
                    ui.button(on_click=lambda field=self.field_provider.active_field: self.delete_field(field)) \
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
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field: self.add_point(field, point, [event.value, point.long])).classes('w-20')
                                    ui.number('longitude', value=geo_point.long, format='%.6f', step=0.1,
                                              on_change=lambda event, point=geo_point, field=self.field_provider.active_field: self.add_point(field, point, [point.lat, event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=geo_point,
                                              field=self.field_provider.active_field: self.add_point(field, point)) \
                                        .props('icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                                    ui.separator()
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                ui.button('', on_click=lambda field=self.field_provider.active_field: self.add_point(field)) \
                                    .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                ui.button('', on_click=lambda field=self.field_provider.active_field: self.remove_point(field)) \
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
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.add_obstacle(field)) \
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
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.add_row(field)) \
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
                        ui.button(on_click=lambda field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.field_provider.remove_obstacle(field, obstacle)).props(
                            'icon=delete color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Delete obstacle')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        if self.coordinate_type == "cartesian":
                            points = self.field_provider.active_object['object'].points(
                                self.field_provider.active_field.reference)
                            for point in points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points(self.field_provider.active_field.reference).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for point in self.field_provider.active_object['object'].cartesian:
                                with ui.row().style("width: 100%;"):

                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].cartesian.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number(
                                        'latitude', value=point[0], format='%.6f', step=0.1,  on_change=lambda event, point=point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.add_obstacle_point(field, obstacle, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=point[1], format='%.6f', step=0.1,
                                        on_change=lambda event, point=point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.add_obstacle_point(field, obstacle, point, [point.lat, event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=point, field=self.field_provider.active_field, obstacle=self.field_provider.active_object['object']: self.add_obstacle_point(field, obstacle, point)).props(
                                        'icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-8')
                                ui.button('', on_click=lambda field=self.field_provider.active_field,
                                          obstacle=self.field_provider.active_object['object']: self.add_obstacle_point(field=field, obstacle=obstacle)).props(
                                    'icon=add color=primary fab-mini flat')
                                ui.button('', on_click=lambda obstacle=self.field_provider.active_object['object']: self.remove_obstacle_point(
                                    obstacle)).props('icon=remove color=warning fab-mini flat')
                elif self.tab == "Rows":
                    with ui.row().style("width: 100%"):
                        ui.icon('spa').props('size=sm color=primary').style(
                            "height: 100%; margin-top:auto; margin-bottom: auto;")
                        with ui.column():
                            ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.move_row(self.field_provider.active_field, row)) .props(
                                'icon=expand_less color=primary fab-mini flat').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                            ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.move_row(self.field_provider.active_field, row, next=True)) .props(
                                'icon=expand_more color=primary fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                        ui.input(value=self.field_provider.active_object['object'].name).on('blur', self.field_provider.invalidate).bind_value(
                            self.field_provider.active_object['object'], 'name').classes('w-32')
                        ui.button(on_click=self.field_provider.active_object['object'].clear_crops).props(
                            'icon=sym_o_compost color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Clear all row crops')
                        ui.button(on_click=lambda row=self.field_provider.active_object['object']: self.field_provider.remove_row(self.field_provider.active_field, row)).props(
                            'icon=delete color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Delete Row')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        if self.coordinate_type == "cartesian":
                            points = self.field_provider.active_object['object'].points(
                                self.field_provider.active_field.reference)
                            for point in points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points(self.field_provider.active_field.reference).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for point in self.field_provider.active_object['object'].points:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leaflet_map.m.set_center(self.field_provider.active_object['object'].cartesian[self.field_provider.active_object['object'].points.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number(
                                        'latitude', value=point[0], format='%.6f', step=0.1,  on_change=lambda event, point=point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.add_row_point(field, row, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=point[1], format='%.6f', step=0.1,
                                        on_change=lambda event, point=point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.add_row_point(field, row, point, [point[0], event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=point, field=self.field_provider.active_field, row=self.field_provider.active_object['object']: self.add_row_point(field, row, point)).props(
                                        'icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2').style('display: block; margin: auto;'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                ui.button('', on_click=lambda field=self.field_provider.active_field,
                                          row=self.field_provider.active_object['object']: self.add_row_point(field, row)) \
                                    .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                ui.button(
                                    '', on_click=lambda row=self.field_provider.active_object['object']: self.remove_row_point(row)) \
                                    .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')

    def get_field_reference(self, field: Field) -> None:
        if self.gnss.device is None:
            self.log.warning('not creating Reference because no GNSS device found')
            rosys.notify('No GNSS device found', 'negative')
            return
        if self.gnss.record.gps_qual != 4:
            self.log.warning('not creating Reference because no RTK fix available')
            rosys.notify('No RTK fix available', 'negative')
            return
        if field.reference_lat is None or field.reference_lon is None:
            ref_lat, ref_lon = self.gnss.get_reference()
            if ref_lat is None or ref_lon is None:
                self.log.warning('not creating Point because no reference position available')
                rosys.notify('No reference position available')
                return
            field.reference_lat = ref_lat
            field.reference_lon = ref_lon
        if self.gnss.reference_lat != field.reference_lat or self.gnss.reference_lon != field.reference_lon:
            self.gnss.set_reference(field.reference_lat, field.reference_lon)

    async def add_point(self, field: Field, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        positioning = GeoPoint.from_list([self.gnss.record.latitude, self.gnss.record.longitude])
        if positioning is None or positioning.lat == 0 or positioning.long == 0:
            rosys.notify("No GNSS position.")
            return
        if not (self.gnss.record.gps_qual == 4 or self.gnss.record.gps_qual == 8):
            rosys.notify("GNSS position is not accurate enough.")
            return
        new_point = positioning
        if point is not None:
            index = field.points.index(point)
            if index == 0:
                self.field_provider.set_reference(field, new_point)
            field.points[index] = new_point
        else:
            if len(field.points) < 1:
                self.field_provider.set_reference(field, new_point)
                self.gnss.set_reference(new_point)
            field.points.append(new_point)
        self.field_provider.invalidate()

    def remove_point(self, field: Field, point: Optional[GeoPoint] = None) -> None:
        if point is not None:
            index = field.points.index(point)
            del field.points[index]
        elif field.points:
            del field.points[-1]
        self.field_provider.invalidate()

    def add_field(self) -> None:
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'field_{len(self.field_provider.fields)+1}', points=[])
        self.field_provider.add_field(field)

    def delete_field(self, field: Field) -> None:
        self.field_provider.remove_field(field)

    def clear_fields(self) -> None:
        self.field_provider.clear_fields()

    def add_obstacle(self, field: Field) -> None:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'obstacle_{len(field.obstacles)+1}', points=[])
        self.field_provider.add_obstacle(field, obstacle)

    async def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            positioning = GeoPoint.from_list([self.gnss.record.latitude, self.gnss.record.longitude])
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not (self.gnss.record.gps_qual == 4 or self.gnss.record.gps_qual == 8):
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = obstacle.points.index(point)
            obstacle.points[index] = new_point
        else:
            obstacle.points.append(new_point)
        assert self.field_provider.active_object
        self.field_provider.select_object(self.field_provider.active_object['object'].id, self.tab)
        self.field_provider.invalidate()

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[GeoPoint] = None) -> None:
        if obstacle.points:
            if point is not None:
                index = obstacle.points.index(point)
                del obstacle.points[index]
            else:
                del obstacle.points[-1]
            assert self.field_provider.active_object
            self.field_provider.select_object(self.field_provider.active_object['object'].id, self.tab)
            self.field_provider.invalidate()

    def add_row(self, field: Field) -> None:
        row = Row(id=f'{str(uuid.uuid4())}', name=f'row_{len(field.rows)+1}', points=[])
        self.field_provider.add_row(field, row)

    def add_row_point(self, field: Field, row: Row, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            positioning = GeoPoint.from_list([self.gnss.record.latitude, self.gnss.record.longitude])
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not (self.gnss.record.gps_qual == 4 or self.gnss.record.gps_qual == 8):
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = row.points.index(point)
            row.points[index] = new_point
        else:
            row.points.append(new_point)
        assert self.field_provider.active_object
        self.field_provider.select_object(self.field_provider.active_object['object'].id, self.tab)
        self.field_provider.invalidate()

    def remove_row_point(self, row: Row, point: Optional[GeoPoint] = None) -> None:
        if row.points:
            if point is not None:
                index = row.points.index(point)
                del row.points[index]
            else:
                del row.points[-1]
            assert self.field_provider.active_object
            self.field_provider.select_object(self.field_provider.active_object['object'].id, self.tab)
            self.field_provider.invalidate()

    def move_row(self, field: Field, row: Row, next: bool = False) -> None:
        index = field.rows.index(row)
        if next:
            if index == len(field.rows)-1:
                field.rows[index], field.rows[0] = field.rows[0], field.rows[index]
            else:
                field.rows[index], field.rows[index+1] = field.rows[index+1], field.rows[index]
        else:
            field.rows[index], field.rows[index-1] = field.rows[index-1], field.rows[index]
        self.field_provider.invalidate()
