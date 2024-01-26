import json
import logging
import uuid
import xml.etree.ElementTree as ET
from typing import Optional, Union

import geopandas as gpd
import rosys
from nicegui import events, ui
from nicegui.events import ValueChangeEventArguments
from shapely.ops import transform

from field_friend.navigation.point_transformation import cartesian_to_wgs84, wgs84_to_cartesian

from ..automations import Field, FieldObstacle, FieldProvider, Row
from ..navigation import Gnss
from .leaflet_map import leaflet_map
from .local_file_picker import local_file_picker
from .operation import operation


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss, leaflet_map: leaflet_map) -> None:
        self.log = logging.getLogger('field_friend.field_planner')
        self.field_provider = field_provider
        self.operation = operation
        self.odometer = odometer
        self.gnss = gnss
        self.leafet_map = leaflet_map

        self.coordinate_type = "cartesian"
        self.COORDINATE_TYPE_CHANGED = rosys.event.Event()
        "switch between displaying cartesian and wgs84 coordiantes."

        self.active_element = None
        self.ELEMENT_SELECTED = rosys.event.Event()
        "a row or obstacle has been selected or deselected."

        self.tab = "Outline"
        self.TAB_CHANGED = rosys.event.Event()

        with ui.row().classes('w-full').style('height: 100%; max-height:100%; width: 100%;'):
            with ui.card().style('width: 48%; max-width: 48%; max-height: 100%; height: 100%;'):
                with ui.row():
                    self.show_coordinate_type_selection()
                    ui.button('Upload Field', on_click=self.upload_field).tooltip(
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

            self.show_element_settings()
            self.ELEMENT_SELECTED.register(self.show_element_settings.refresh)
            self.COORDINATE_TYPE_CHANGED.register(self.show_element_settings.refresh)

    def handle_coordinate_type_change(self, e: events.GenericEventArguments) -> None:
        self.coordinate_type = e.value
        self.COORDINATE_TYPE_CHANGED.emit()

    def select_element(self, id: Optional[str] = None) -> None:
        if id is not None:
            if self.tab == "Obstacles":
                for obstacle in self.field_provider.active_field.obstacles:
                    if id == obstacle.id:
                        self.active_element = obstacle
            elif self.tab == "Rows":
                for row in self.field_provider.active_field.rows:
                    if id == row.id:
                        self.active_element = row
            else:
                self.active_element = None
        else:
            self.active_element = None
        self.ELEMENT_SELECTED.emit()

    def set_tab(self, e: events.GenericEventArguments) -> None:
        self.tab = e.value
        self.select_element(None)
        self.TAB_CHANGED.emit()

# TODO when a field in the table is selected change the color in the map jump to it and add all obstacles and rows to the map
    def table_selected(self, selection):
        if len(selection.selection) > 0:
            for field in self.field_provider.fields:
                if field.id == selection.selection[0]['id']:
                    self.field_provider.select_field(field)
                    if len(field.outline_wgs84) > 0:
                        self.gnss.set_reference(field.outline_wgs84[0][0], field.outline_wgs84[0][1])
        else:
            self.field_provider.select_field(None)

    def show_coordinate_type_selection(self) -> None:
        ui.select(["cartesian", "WGS84"], value=self.coordinate_type, on_change=self.handle_coordinate_type_change)

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
                    'boundary': f'{len(field.outline_wgs84)} points',
                    'obstacles': f'{len(field.obstacles)}',
                    'rows': f'{len(field.rows)}'}
            rows.append(data)
        field_table = ui.table(columns=columns, rows=rows, row_key='id',  selection='single',
                               on_select=self.table_selected, pagination=4).style('width: 98%; max-width: 98%;')
        columns[0]['classes'] = 'hidden'
        columns[0]['headerClasses'] = 'hidden'
        field_table.update()

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
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.field_provider.active_field.outline_wgs84[self.field_provider.active_field.outline.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        elif self.coordinate_type == "WGS84":
                            for point in self.field_provider.active_field.outline_wgs84:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.field_provider.active_field.outline_wgs84[self.field_provider.active_field.outline_wgs84.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number(
                                        'latitude', value=point[0], format='%.6f', step=0.1,  on_change=lambda event, point=point, field=self.field_provider.active_field: self.add_point(field, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=point[1], format='%.6f', step=0.1,
                                        on_change=lambda event, point=point, field=self.field_provider.active_field: self.add_point(field, point, [point[0], event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=point, field=self.field_provider.active_field: self.add_point(field, point)).props(
                                        'icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
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
                            if self.active_element and self.tab == "Obstacles":
                                obstacle_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.select_element(event.value), value=self.active_element.id)
                            else:
                                obstacle_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.select_element(event.value))
                        with ui.row().classes('items-center mt-3').style("width: 100%"):
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.add_obstacle(field)) \
                                .props('color=primary outline').style("width: 100%")
                    with ui.tab_panel('Rows'):
                        with ui.row().style("width: 100%;"):
                            radio_el = {}
                            for row in self.field_provider.active_field.rows:
                                radio_el[row.id] = row.name
                            if self.active_element and self.tab == "Rows":
                                row_radio = ui.radio(
                                    radio_el, on_change=lambda event: self.select_element(event.value), value=self.active_element.id)
                            else:
                                row_radio = ui.radio(radio_el, on_change=lambda event: self.select_element(event.value))

                        with ui.row().classes('items-center mt-3').style("width: 100%"):
                            ui.button(icon='add', on_click=lambda field=self.field_provider.active_field: self.add_row(field)) \
                                .props('color=primary outline').style("width: 100%")

    @ui.refreshable
    def show_element_settings(self) -> None:
        with ui.card().style('width: 24%; max-height: 100%; height: 100%;'):
            if self.active_element is None:
                with ui.column().style('display: block; margin: auto;'):
                    ui.icon('extension').props('size=lg color=primary').style('display: block; margin: auto;')
                    ui.label("select an element").style('display: block; margin: auto; color: #6E93D6;')
            else:
                if self.tab == "Obstacles":
                    with ui.row().style("width: 100%;"):
                        ui.icon('dangerous').props('size=sm color=primary').style(
                            "display:block; margin-top:auto; margin-bottom: auto;")
                        ui.input(
                            'Obstacle name', value=f'{self.active_element.name}').on('blur', self.field_provider.invalidate).bind_value(
                            self.active_element, 'name').classes('w-32')
                        ui.button(on_click=lambda field=self.field_provider.active_field, obstacle=self.active_element: self.remove_obstacle(field, obstacle)).props(
                            'icon=delete color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Delete obstacle')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        if self.coordinate_type == "cartesian":
                            for point in self.active_element.points([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon]):
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.active_element.points_wgs84[self.active_element.points([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon]).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for point in self.active_element.points_wgs84:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.active_element.points_wgs84[self.active_element.points_wgs84.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number(
                                        'latitude', value=point[0], format='%.6f', step=0.1,  on_change=lambda event, point=point, field=self.field_provider.active_field, obstacle=self.active_element: self.add_obstacle_point(field, obstacle, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=point[1], format='%.6f', step=0.1,
                                        on_change=lambda event, point=point, field=self.field_provider.active_field, obstacle=self.active_element: self.add_obstacle_point(field, obstacle, point, [point[0], event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=point, field=self.field_provider.active_field, obstacle=self.active_element: self.add_obstacle_point(field, obstacle, point)).props(
                                        'icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-8')
                                ui.button('', on_click=lambda field=self.field_provider.active_field,
                                          obstacle=self.active_element: self.add_obstacle_point(field=field, obstacle=obstacle)).props(
                                    'icon=add color=primary fab-mini flat')
                                ui.button('', on_click=lambda obstacle=self.active_element: self.remove_obstacle_point(
                                    obstacle)).props('icon=remove color=warning fab-mini flat')
                elif self.tab == "Rows":
                    with ui.row().style("width: 100%"):
                        ui.icon('spa').props('size=sm color=primary').style(
                            "height: 100%; margin-top:auto; margin-bottom: auto;")
                        with ui.column():
                            ui.button(on_click=lambda row=self.active_element: self.move_row(self.field_provider.active_field, row)) .props(
                                'icon=expand_less color=primary fab-mini flat').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                            ui.button(on_click=lambda row=self.active_element: self.move_row(self.field_provider.active_field, row, next=True)) .props(
                                'icon=expand_more color=primary fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto; margin-left: 0; margin-right: 0;")
                        ui.input(value=self.active_element.name).on('blur', self.field_provider.invalidate).bind_value(
                            self.active_element, 'name').classes('w-32')
                        ui.button(on_click=lambda row=self.active_element: self.remove_row(self.field_provider.active_field, row)).props(
                            'icon=delete color=warning fab-mini flat').classes('ml-auto').style("display:block; margin-top:auto; margin-bottom: auto;").tooltip('Delete Row')
                    with ui.column().style("display: block; overflow: auto; width: 100%"):
                        if self.coordinate_type == "cartesian":
                            for point in self.active_element.points([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon]):
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.active_element.points_wgs84[self.active_element.points([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon]).index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.label(f'x: {float("{:.2f}".format(point.x))}')
                                    ui.label(f'y: {float("{:.2f}".format(point.y))}')
                        else:
                            for point in self.active_element.points_wgs84:
                                with ui.row().style("width: 100%;"):
                                    ui.button(on_click=lambda point=point: self.leafet_map.m.set_center(self.active_element.points_wgs84[self.active_element.points_wgs84.index(point)])).props(
                                        'icon=place color=primary fab-mini flat').tooltip('center map on point').classes('ml-0')
                                    ui.number(
                                        'latitude', value=point[0], format='%.6f', step=0.1,  on_change=lambda event, point=point, field=self.field_provider.active_field, row=self.active_element: self.add_row_point(field, row, point, [event.value, point[1]])).classes('w-20')
                                    ui.number(
                                        'longitude', value=point[1], format='%.6f', step=0.1,
                                        on_change=lambda event, point=point, field=self.field_provider.active_field, row=self.active_element: self.add_row_point(field, row, point, [point[0], event.value])).classes('w-20')
                                    ui.button(on_click=lambda point=point, field=self.field_provider.active_field, row=self.active_element: self.add_row_point(field, row, point)).props(
                                        'icon=edit_location_alt color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                            with ui.row().classes('items-center mt-2').style('display: block; margin: auto;'):
                                ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                ui.button('', on_click=lambda field=self.field_provider.active_field, row=self.active_element: self.add_row_point(field, row)) \
                                    .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                ui.button('', on_click=lambda row=self.active_element: self.remove_row_point(row)) \
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

    def add_point(self, field: Field, point: Optional[list] = None, new_point: Optional[list] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = field.outline_wgs84.index(point)
            if new_point is None:
                # TODO in allen Funktionen wäre es bestimmt sinnvoller die Koords direkt vom GNSS zu holen
                # muss dann noch darauf gewaretet werden, dass das signal eine hohe genauigkeit hat und dann erst gespeichert wirdd
                # damit dann noch feedback im UI
                new_point = cartesian_to_wgs84([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon], [
                                               self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            if index == 0:
                field.reference_lat = new_point[0]
                field.reference_lon = new_point[1]
            field.outline_wgs84[index] = new_point
        else:
            point = cartesian_to_wgs84([self.field_provider.active_field.reference_lat, self.field_provider.active_field.reference_lon], [
                self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            field.outline.append(point)
        self.field_provider.invalidate()

    def remove_point(self, field: Field, point: Optional[list] = None) -> None:
        if point is not None:
            # FIXME: wenn der erste und der letzte Punkt wie in einem Polygon gleich sind,
            # dann kann  es durch die suche nach dem ersten index zu problemen kommen, wenn der letzte punkt gelöscht werden soll
            index = field.outline_wgs84.index(point)
            del field.outline_wgs84[index]
        elif field.outline_wgs84 != []:
            del field.outline_wgs84[-1]
        self.field_provider.invalidate()

    def swap_coordinates(self, lon, lat):
        return lat, lon

    def extract_coordinates_kml(self, kml_path):
        gdf = gpd.read_file(kml_path, drivr="KML")
        coordinates = []
        for c in gdf["geometry"][0].coords:
            lat = c[1]
            lon = c[0]
            coordinates.append([lat, lon])
        return coordinates

    def extract_coordinates_xml(self, xml_path):
        tree = ET.parse(xml_path)
        root = tree.getroot()
        coordinates = []
        for geo_data in root.findall('.//LSG'):
            for point in geo_data.findall('.//PNT'):
                lat = float(point.attrib['C'])
                lon = float(point.attrib['D'])
                coordinates.append([lat, lon])
        return coordinates

    async def upload_field(self) -> None:
        result = await local_file_picker('~', multiple=True)
        coordinates = []
        if result is None:
            rosys.notify("You can only upload the following file formates: .shp, .kml and .xml.")
            return
        elif result[0][-3:].casefold() == "shp":
            gdf = gpd.read_file(result[0])
            gdf['geometry'] = gdf['geometry'].apply(lambda geom: transform(self.swap_coordinates, geom))
            feature = json.loads(gdf.to_json())
            coordinates = feature["features"][0]["geometry"]["coordinates"][0]
        elif result[0][-3:].casefold() == "kml":
            coordinates = self.extract_coordinates_kml(result[0])
        elif result[0][-3:].casefold() == "xml":
            coordinates = self.extract_coordinates_xml(result[0])
        else:
            rosys.notify("You can only upload the following file formates: .shp, .kml and .xml.")
            return
        reference_point = coordinates[0]
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'{new_id}', outline_wgs84=coordinates,
                      reference_lat=reference_point[0], reference_lon=reference_point[1])
        self.field_provider.add_field(field)
        return

    def add_field(self) -> None:
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'{new_id}', outline_wgs84=[])
        self.field_provider.add_field(field)

    def delete_field(self, field: Field) -> None:
        self.field_provider.remove_field(field)

    def clear_fields(self) -> None:
        self.field_provider.clear_fields()

    def add_obstacle(self, field: Field) -> None:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}', points_wgs84=[])
        self.field_provider.add_obstacle(field, obstacle)

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        self.field_provider.remove_obstacle(field, obstacle)
        self.active_element = None
        self.ELEMENT_SELECTED.emit()

    def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: Optional[list] = None, new_point: Optional[list] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = obstacle.points_wgs84.index(point)
            if new_point is None:
                new_point = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [
                    self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            obstacle.points_wgs84[index] = new_point
        else:
            point = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [
                self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            obstacle.points_wgs84.append(point)
        self.field_provider.invalidate()
        self.select_element(self.active_element.id)

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[list] = None) -> None:
        if point is not None:
            index = obstacle.points_wgs84.index(point)
            del obstacle.points_wgs84[index]
        elif obstacle.points_wgs84 != []:
            del obstacle.points_wgs84[-1]
        self.field_provider.invalidate()
        self.select_element(self.active_element.id)

    def add_row(self, field: Field) -> None:
        row = Row(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}', points_wgs84=[])
        self.field_provider.add_row(field, row)

    def remove_row(self, field: Field, row: Row) -> None:
        self.field_provider.remove_row(field, row)
        self.select_element(None)

    def add_row_point(self, field: Field, row: Row, point: Optional[list] = None, new_point: Optional[list] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = row.points_wgs84.index(point)
            if new_point is None:
                new_point = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [
                    self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            row.points_wgs84[index] = new_point
        else:
            point = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [
                self.odometer.prediction.point.x, self.odometer.prediction.point.y])
            row.points_wgs84.append(point)
        self.field_provider.invalidate()
        self.select_element(self.active_element.id)

    def remove_row_point(self, row: Row, point: Optional[list] = None) -> None:
        if point is not None:
            index = row.points_wgs84.index(point)
            del row.points_wgs84[index]
        elif row.points_wgs84 != []:
            del row.points_wgs84[-1]
        self.field_provider.invalidate()
        self.select_element(self.active_element.id)

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
