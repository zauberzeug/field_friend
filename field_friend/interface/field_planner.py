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

from field_friend.navigation.point_transformation import wgs84_to_cartesian

from ..automations import Field, FieldObstacle, FieldProvider, Row
from ..navigation import Gnss
from .local_file_picker import local_file_picker
from .operation import operation


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.field_planner')
        self.field_provider = field_provider
        self.operation = operation
        self.odometer = odometer
        self.gnss = gnss
        self.active_element = None
        self.tab = ""
        self.ELEMENT_SELECTED = rosys.event.Event()

        with ui.row().classes('w-full').style('height: 100%; max-height:100%; width: 100%;'):
            with ui.card().style('width: 48%; max-width: 48%; max-height: 100%; height: 100%;'):
                with ui.row():
                    # ui.select()
                    ui.button('Upload Field', on_click=self.upload_field).tooltip(
                        'Upload a file with field boundaries. Supported file formates: KML, XML and Shape')
                    ui.button('Add field', on_click=self.add_field).tooltip('Add a new field')
                    ui.button('Clear fields', on_click=self.clear_fields).props(
                        'outline color=warning').tooltip('Delete all fields')
                with ui.row().style('width: 100%;'):
                    self.show_field_table()
                    self.field_provider.FIELDS_CHANGED.register(self.show_field_table.refresh)
            self.show_field_settings()
            self.field_provider.FIELD_SELECTED.register(self.show_field_settings.refresh)
            self.field_provider.FIELDS_CHANGED.register(self.show_field_settings.refresh)

            self.show_element_settings()
            self.ELEMENT_SELECTED.register(self.show_element_settings.refresh)

    def select_element(self, e: events.GenericEventArguments) -> None:
        if self.tab == "Obstacles":
            for obstacle in self.field_provider.active_field.obstacles:
                if e.value == obstacle.id:
                    self.active_element = obstacle
        elif self.tab == "Rows":
            for row in self.field_provider.active_field.rows:
                if e.value == row.id:
                    self.active_element = row
        else:
            self.active_element = None
        self.ELEMENT_SELECTED.emit()

    def set_tab(self, e: events.GenericEventArguments) -> None:
        self.tab = e.value

    def row_selected(self, selection):
        if len(selection.selection) > 0:
            for field in self.field_provider.fields:
                if field.id == selection.selection[0]['id']:
                    self.field_provider.select_field(field)
        else:
            self.field_provider.select_field(None)

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
                    'boundary': f'{len(field.outline)} points',
                    'obstacles': f'{len(field.obstacles)}',
                    'rows': f'{len(field.rows)}'}
            rows.append(data)
        field_table = ui.table(columns=columns, rows=rows, row_key='id',  selection='single',
                               on_select=self.row_selected, pagination=4).style('width: 98%; max-width: 98%;')
        columns[0]['classes'] = 'hidden'
        columns[0]['headerClasses'] = 'hidden'
        field_table.update()

    @ui.refreshable
    def show_field_settings(self) -> None:
        if self.field_provider.active_field != None:
            with ui.card().style('width: 23%; max-height: 100%; height: 100%;'):
                with ui.row().classes('items-center'):
                    ui.icon('fence').props('size=lg color=primary')
                    ui.input(
                        'Field name', value=f'{self.field_provider.active_field.name}').on('blur', self.field_provider.invalidate).bind_value(
                        self.field_provider.active_field, 'name').classes('w-32')
                    ui.button(on_click=lambda field=self.field_provider.active_field: self.delete_field(field)) \
                        .props('icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete field')
                with ui.tabs().style('width: 100%;') as self.tabs:
                    ui.tab('Outline', 'Outline')
                    ui.tab('Obstacles', 'Obstacles')
                    ui.tab('Rows', 'Rows')
                with ui.tab_panels(self.tabs, value='Outline', on_change=self.set_tab).style('width: 100%;') as self.panels:
                    with ui.tab_panel('Outline').style('width: 100%;'):
                        for point in self.field_provider.active_field.outline:
                            with ui.row().classes('items-center'):
                                ui.button(on_click=lambda field=self.field_provider.active_field, point=point: self.add_point(field, point)).props(
                                    'icon=place color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                                ui.number(
                                    'x', value=point.x, format='%.2f', step=0.1,
                                    on_change=self.field_provider.invalidate).bind_value(
                                    point, 'x').classes('w-16')
                                ui.number(
                                    'y', value=point.y, format='%.2f', step=0.1,
                                    on_change=self.field_provider.invalidate).bind_value(
                                    point, 'y').classes('w-16')
                        with ui.row().classes('items-center mt-2'):
                            ui.icon('place').props('size=sm color=grey').classes('ml-2')
                            ui.button('', on_click=lambda field=self.field_provider.active_field: self.add_point(field)) \
                                .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                            ui.button('', on_click=lambda field=self.field_provider.active_field: self.remove_point(field)) \
                                .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')

                    with ui.tab_panel('Obstacles'):
                        with ui.row().classes('items-center'):
                            columns_obstacles = [
                                {'name': 'id', 'label': 'id', 'field': 'id', 'required': True, 'align': 'center'},
                                {'name': 'name', 'label': 'Name', 'field': 'name', 'required': True, 'align': 'left'}
                            ]
                            rows_obstacles = []
                            radio = {}
                            for obstacle in self.field_provider.active_field.obstacles:
                                data = {'id': obstacle.id,
                                        'name': obstacle.name}
                                rows_obstacles.append(data)
                                radio[obstacle.id] = obstacle.name
                            obstacle_radio = ui.radio(radio, on_change=self.select_element)
                            """ui.table(columns=columns_obstacles, rows=rows_obstacles, row_key='id',  selection='single',
                                     on_select=self.row_selected, pagination=4).style('width: 98%; max-width: 98%;')
                            columns_obstacles[0]['classes'] = 'hidden'
                            columns_obstacles[0]['headerClasses'] = 'hidden'
                            """
                            """
                            for obstacle in self.field_provider.active_field.obstacles:
                                with ui.card().classes('items-stretch'):
                                    with ui.row().classes('items-center'):
                                        ui.icon('block').props('size=sm color=primary')
                                        ui.input(
                                            'Obstacle name', value=f'{obstacle.name}').on('blur', self.field_provider.invalidate).bind_value(
                                            obstacle, 'name').classes('w-32')
                                        ui.button(on_click=lambda field=self.field_provider.active_field, obstacle=obstacle: self.remove_obstacle(field, obstacle)).props(
                                            'icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete obstacle')
                                    for point in obstacle.points:
                                        with ui.row().classes('items-center'):
                                            ui.button(
                                                on_click=lambda field=self.field_provider.active_field, point=point: self.add_point(field, point)).props(
                                                'icon=place color=primary fab-mini flat').tooltip('Relocate point').classes('ml-6')
                                            ui.number(
                                                'x', value=point.x, format='%.2f', step=0.1,
                                                on_change=self.field_provider.invalidate).bind_value(
                                                point, 'x').classes('w-16')
                                            ui.number(
                                                'y', value=point.y, format='%.2f', step=0.1,
                                                on_change=self.field_provider.invalidate).bind_value(
                                                point, 'y').classes('w-16')
                                    with ui.row().classes('items-center mt-2'):
                                        ui.icon('place').props('size=sm color=grey').classes('ml-8')
                                    ui.button('', on_click=lambda field=self.field_provider.active_field,
                                              obstacle=obstacle: self.add_obstacle_point(field, obstacle)).props(
                                        'icon=add color=primary fab-mini flat')
                                    ui.button('', on_click=lambda obstacle=obstacle: self.remove_obstacle_point(
                                        obstacle)).props('icon=remove color=warning fab-mini flat')
                                    """
                        with ui.row().classes('items-center mt-3'):
                            ui.icon('block').props('size=sm color=grey')
                            ui.button('ADD OBSTACLE', on_click=lambda field=self.field_provider.active_field: self.add_obstacle(field)) \
                                .props('color=primary outline')

                    with ui.tab_panel('Rows'):
                        with ui.row():
                            for row in self.field_provider.active_field.rows:
                                with ui.card().classes('items-stretch'):
                                    with ui.row().classes('items-center'):
                                        ui.icon('spa').props('size=sm color=primary')
                                        ui.input(
                                            'Row name', value=row.id).on('blur', self.field_provider.invalidate).bind_value(
                                            row, 'id').classes('w-32')
                                        ui.button(on_click=lambda row=row: self.remove_row(self.field_provider.active_field, row)) \
                                            .props('icon=delete color=warning fab-mini flat')
                                        if row != self.field_provider.active_field.rows[0]:
                                            ui.button(on_click=lambda row=row: self.move_row(self.field_provider.active_field, row)) .props(
                                                'icon=navigate_before color=primary fab-mini flat')
                                        if row != self.field_provider.active_field.rows[-1]:
                                            ui.button(on_click=lambda row=row: self.move_row(self.field_provider.active_field, row, next=True)) .props(
                                                'icon=navigate_next color=primary fab-mini flat').classes('ml-auto')
                                    for point in row.points:
                                        with ui.row().classes('items-center'):
                                            ui.button(on_click=lambda field=self.field_provider.active_field, row=row, point=point: self.add_row_point(field, row, point)).props(
                                                'icon=place color=primary fab-mini flat').tooltip('Relocate point').classes('ml-0')
                                            ui.number(
                                                'x', value=point.x, format='%.2f', step=0.1,
                                                on_change=self.field_provider.invalidate).bind_value(
                                                point, 'x').classes('w-16')
                                            ui.number(
                                                'y', value=point.y, format='%.2f', step=0.1,
                                                on_change=self.field_provider.invalidate).bind_value(
                                                point, 'y').classes('w-16')
                                    with ui.row().classes('items-center mt-2'):
                                        ui.icon('place').props('size=sm color=grey').classes('ml-2')
                                        ui.button('', on_click=lambda field=self.field_provider.active_field, row=row: self.add_row_point(field, row)) \
                                            .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                        ui.button('', on_click=lambda row=row: self.remove_row_point(row)) \
                                            .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')
                                    with ui.button('CLEAR ROW CROPS', on_click=lambda row=row: self.clear_row_crops(row=row)) \
                                            .props('color=warning outline'):
                                        pass

                        with ui.row().classes('items-center mt-3'):
                            ui.icon('spa').props('size=sm color=grey')
                            ui.button('ADD ROW', on_click=lambda field=self.field_provider.active_field: self.add_row(field)) \
                                .props('color=primary outline')

    @ui.refreshable
    def show_element_settings(self) -> None:
        if self.active_element != None:
            with ui.card().style('width: 23%; max-height: 100%; height: 100%;'):
                ui.label("work in progress")
                ui.label(self.active_element.name)

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

    # TODO add the WGS84 coordinates. so that a new point or updated will be in outline and outlineWGS84
    def add_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = field.outline.index(point)
            print(f'🟥 {index}')
            # indexwgs = field.outline_wgs84.index(point)
            # print(f'🟦 {indexwgs}')
            new_point = self.odometer.prediction.point
            field.outline[index] = new_point
        else:
            point = self.odometer.prediction.point
            field.outline.append(point)
            # field.outline_wgs84.append(point)
        self.field_provider.invalidate()

    def remove_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = field.outline.index(point)
            del field.outline[index]
        elif field.outline != []:
            del field.outline[-1]
        self.field_provider.invalidate()
        self.panels.set_value('Outline')

    # Define a custom transformation function to swap coordinates
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
        coordinates_cart = []
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
        for point in coordinates:
            cart_coords = wgs84_to_cartesian(reference_point, point)
            current_point = rosys.geometry.Point(x=cart_coords[0], y=cart_coords[1])
            coordinates_cart.append(current_point)
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'{new_id}', outline=coordinates_cart,
                      outline_wgs84=coordinates, reference_lat=reference_point[0], reference_lon=reference_point[1])
        self.field_provider.add_field(field)
        return

    def add_field(self) -> None:
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'{new_id}')
        self.field_provider.add_field(field)
        self.panels.set_value('Outline')

    def delete_field(self, field: Field) -> None:
        self.field_provider.remove_field(field)
        self.panels.set_value('Outline')

    def clear_fields(self) -> None:
        self.field_provider.clear_fields()
        self.panels.set_value('Outline')

    def add_obstacle(self, field: Field) -> None:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}')
        self.field_provider.add_obstacle(field, obstacle)
        self.panels.set_value('Obstacles')

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        self.field_provider.remove_obstacle(field, obstacle)
        self.panels.set_value('Obstacles')

    def add_obstacle_point(
            self, field: Field, obstacle: FieldObstacle, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = obstacle.points.index(point)
            point = self.odometer.prediction.point
            obstacle.points[index] = point
        else:
            point = self.odometer.prediction.point
            obstacle.points.append(point)
        self.field_provider.invalidate()
        self.panels.set_value('Obstacles')

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = obstacle.points.index(point)
            del obstacle.points[index]
        elif obstacle.points != []:
            del obstacle.points[-1]
        self.field_provider.invalidate()
        self.panels.set_value('Obstacles')

    def add_row(self, field: Field) -> None:
        row = Row(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}')
        self.field_provider.add_row(field, row)
        self.panels.set_value('Rows')

    def remove_row(self, field: Field, row: Row) -> None:
        self.field_provider.remove_row(field, row)
        self.panels.set_value('Rows')

    def add_row_point(self, field: Field, row: Row, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = row.points.index(point)
            point = self.odometer.prediction.point
            row.points[index] = point
        else:
            point = self.odometer.prediction.point
            row.points.append(point)
        self.field_provider.invalidate()
        self.panels.set_value('Rows')

    def remove_row_point(self, row: Row, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = row.points.index(point)
            del row.points[index]
        elif row.points != []:
            del row.points[-1]
        self.field_provider.invalidate()
        self.panels.set_value('Rows')

    def move_row(self, field: Field, row: Row, next: bool = False) -> None:
        index = field.rows.index(row)
        if next:
            field.rows[index], field.rows[index+1] = field.rows[index+1], field.rows[index]
        else:
            field.rows[index], field.rows[index-1] = field.rows[index-1], field.rows[index]
        self.field_provider.invalidate()
        self.panels.set_value('Rows')

    def clear_row_crops(self, row: Row) -> None:
        row.crops = []
        self.field_provider.invalidate()
        self.panels.set_value('Rows')
