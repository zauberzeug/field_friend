import logging
import uuid
from typing import Optional

import rosys
from nicegui import ui

from ..automations import Field, FieldObstacle, FieldProvider, Row
from ..navigation import Gnss

import xml.etree.ElementTree as ET
from .local_file_picker import local_file_picker
import geopandas as gpd
from shapely.ops import transform
import json
from geographiclib.geodesic import Geodesic
import numpy as np


class field_planner:

    def __init__(self, field_provider: FieldProvider, odometer: rosys.driving.Odometer, gnss: Gnss) -> None:
        self.log = logging.getLogger('field_friend.field_planner')
        self.field_provider = field_provider
        self.odometer = odometer
        self.gnss = gnss

        with ui.card():
            with ui.row():
                ui.button('Upload Field Boundaries', on_click=self.upload_field).tooltip(
                    'Upload a file with field boundaries. Supported file formates: KML, XML and Shape')
                ui.button('Add field', on_click=self.add_field).tooltip('Add a new field')
                ui.button('Clear fields', on_click=self.clear_fields).props(
                    'outline color=warning').tooltip('Delete all fields')
            with ui.row():
                self.show_field_settings()

    @ui.refreshable
    def show_field_settings(self) -> None:
        for field in self.field_provider.fields:
            with ui.card():
                with ui.row().classes('items-center'):
                    ui.icon('fence').props('size=lg color=primary')
                    ui.input(
                        'Field name', value=f'{field.id}', on_change=self.field_provider.invalidate).bind_value(
                        field, 'id').classes('w-32')
                    ui.button(on_click=lambda field=field: self.delete_field(field)) \
                        .props('icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete field')
                with ui.tabs() as self.tabs:
                    ui.tab('Outline', 'Outline')
                    ui.tab('Obstacles', 'Obstacles')
                    ui.tab('Rows', 'Rows')
                with ui.tab_panels(self.tabs, value='Outline') as self.panels:
                    with ui.tab_panel('Outline'):
                        for point in field.outline:
                            with ui.row().classes('items-center'):
                                ui.button(on_click=lambda field=field, point=point: self.add_point(field, point)).props(
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
                            ui.button('', on_click=lambda field=field: self.add_point(field)) \
                                .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                            ui.button('', on_click=lambda field=field: self.remove_point(field)) \
                                .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')

                    with ui.tab_panel('Obstacles'):
                        with ui.row().classes('items-center'):
                            for obstacle in field.obstacles:
                                with ui.card().classes('items-stretch'):
                                    with ui.row().classes('items-center'):
                                        ui.icon('block').props('size=sm color=primary')
                                        ui.input(
                                            'Obstacle name', value=f'{obstacle.id}').bind_value(
                                            obstacle, 'id').classes('w-32')
                                        ui.button(on_click=lambda field=field, obstacle=obstacle: self.remove_obstacle(field, obstacle)).props(
                                            'icon=delete color=warning fab-mini flat').classes('ml-auto').tooltip('Delete obstacle')
                                    for point in obstacle.points:
                                        with ui.row().classes('items-center'):
                                            ui.button(
                                                on_click=lambda field=field, point=point: self.add_point(field, point)).props(
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
                                    ui.button('', on_click=lambda field=field,
                                              obstacle=obstacle: self.add_obstacle_point(field, obstacle)).props(
                                        'icon=add color=primary fab-mini flat')
                                    ui.button('', on_click=lambda obstacle=obstacle: self.remove_obstacle_point(
                                        obstacle)).props('icon=remove color=warning fab-mini flat')

                        with ui.row().classes('items-center mt-3'):
                            ui.icon('block').props('size=sm color=grey')
                            ui.button('ADD OBSTACLE', on_click=lambda field=field: self.add_obstacle(field)) \
                                .props('color=primary outline')

                    with ui.tab_panel('Rows'):
                        with ui.row():
                            for row in field.rows:
                                with ui.card().classes('items-stretch'):
                                    with ui.row().classes('items-center'):
                                        ui.icon('spa').props('size=sm color=primary')
                                        ui.input(
                                            'Row name', value=row.id, on_change=self.field_provider.invalidate).bind_value(
                                            row, 'id').classes('w-32')
                                        ui.button(on_click=lambda row=row: self.remove_row(field, row)) \
                                            .props('icon=delete color=warning fab-mini flat')
                                        if row != field.rows[0]:
                                            ui.button(on_click=lambda row=row: self.move_row(field, row)) .props(
                                                'icon=navigate_before color=primary fab-mini flat')
                                        if row != field.rows[-1]:
                                            ui.button(on_click=lambda row=row: self.move_row(field, row, next=True)) .props(
                                                'icon=navigate_next color=primary fab-mini flat').classes('ml-auto')
                                    for point in row.points:
                                        with ui.row().classes('items-center'):
                                            ui.button(on_click=lambda field=field, row=row, point=point: self.add_row_point(field, row, point)).props(
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
                                        ui.button('', on_click=lambda field=field, row=row: self.add_row_point(field, row)) \
                                            .props('icon=add color=primary fab-mini flat').tooltip('Add point')
                                        ui.button('', on_click=lambda row=row: self.remove_row_point(row)) \
                                            .props('icon=remove color=warning fab-mini flat').tooltip('Remove point')
                                    with ui.button('CLEAR ROW CROPS', on_click=lambda row=row: self.clear_row_crops(row=row)) \
                                            .props('color=warning outline'):
                                        pass

                        with ui.row().classes('items-center mt-3'):
                            ui.icon('spa').props('size=sm color=grey')
                            ui.button('ADD ROW', on_click=lambda field=field: self.add_row(field)) \
                                .props('color=primary outline')

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

    def add_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if self.gnss.device != 'simulation':
            self.get_field_reference(field)
        if point is not None:
            index = field.outline.index(point)
            new_point = self.odometer.prediction.point
            field.outline[index] = new_point
        else:
            point = self.odometer.prediction.point
            field.outline.append(point)
        self.field_provider.invalidate()
        self.show_field_settings.refresh()

    def remove_point(self, field: Field, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = field.outline.index(point)
            del field.outline[index]
        elif field.outline != []:
            del field.outline[-1]
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
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
        ui.notify(f'You chose {result}')
        print(result[0][-3:])
        coordinates = []
        coordinates_carth = []
        # TODO: leichte Abweichungen in den vermeindlich gleichen Datensätzen in verschiedenen Dateiformaten? Sind Ursprungsdaten schon verschieden?
        if result[0][-3:].casefold() == "shp":
            gdf = gpd.read_file(result[0])
            gdf['geometry'] = gdf['geometry'].apply(lambda geom: transform(self.swap_coordinates, geom))
            feature = json.loads(gdf.to_json())
            coordinates = feature["features"][0]["geometry"]["coordinates"][0]
        elif result[0][-3:].casefold() == "kml":
            coordinates = self.extract_coordinates_kml(result[0])
        elif result[0][-3:].casefold() == "xml":
            print("🥺" + str(result[0]))
            coordinates = self.extract_coordinates_xml(result[0])
        else:
            # TODO: throw exception or notification
            pass
        reference_point = coordinates[0]
        for point in coordinates:
            # calc the carthesian coordiantes
            r = Geodesic.WGS84.Inverse(reference_point[0], reference_point[1], point[0], point[1])
            s = r['s12']
            a = -np.deg2rad(r['azi1'])
            x = s * np.cos(a)
            y = s * np.sin(a)
            current_point = rosys.geometry.Point(x=x, y=y)
            coordinates_carth.append(current_point)
        # funktioniert aktuell nur für files mit nur einem einzigen Polygon und nicht mit Features die mehrere Polygone enthalten, keine exception bisher
        # die Koordinaten umwandlen als karthesische Koordinaten und dann als Points speichern
        # den ersten Punkt als Referenzpunkt nehmen oder einen anderen

        field = Field(id=f'{str(uuid.uuid4())}', outline=coordinates_carth,
                      outline_wgs84=coordinates)
        self.field_provider.add_field(field)
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

        # Feld anlegen
        # diesem feld die Punkte übergeben und damit direkt die Outline anzeigen
        # diese wird dann im field Planner angezeigt
        # in leaflet muss dann nurnoch dem event für neues feld subscribed werden.

        # weitere ideen:
        # die Outline in einem Field sowohl karthesisch als auch
        # im Field speichern ob dieses gerade als visible oder nicht markiert ist und damit dann in leaflet angezeigt wird oder nicht
        return

    def add_field(self) -> None:
        field = Field(id=f'{str(uuid.uuid4())}')
        self.field_provider.add_field(field)
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def delete_field(self, field: Field) -> None:
        self.field_provider.remove_field(field)
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def clear_fields(self) -> None:
        self.field_provider.clear_fields()
        self.show_field_settings.refresh()
        self.panels.set_value('Outline')

    def add_obstacle(self, field: Field) -> None:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}')
        self.field_provider.add_obstacle(field, obstacle)
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        self.field_provider.remove_obstacle(field, obstacle)
        self.show_field_settings.refresh()
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
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = obstacle.points.index(point)
            del obstacle.points[index]
        elif obstacle.points != []:
            del obstacle.points[-1]
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Obstacles')

    def add_row(self, field: Field) -> None:
        row = Row(id=f'{str(uuid.uuid4())}')
        self.field_provider.add_row(field, row)
        self.show_field_settings.refresh()
        self.panels.set_value('Rows')

    def remove_row(self, field: Field, row: Row) -> None:
        self.field_provider.remove_row(field, row)
        self.show_field_settings.refresh()
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
        self.show_field_settings.refresh()
        self.panels.set_value('Rows')

    def remove_row_point(self, row: Row, point: Optional[rosys.geometry.Point] = None) -> None:
        if point is not None:
            index = row.points.index(point)
            del row.points[index]
        elif row.points != []:
            del row.points[-1]
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Rows')

    def move_row(self, field: Field, row: Row, next: bool = False) -> None:
        index = field.rows.index(row)
        if next:
            field.rows[index], field.rows[index+1] = field.rows[index+1], field.rows[index]
        else:
            field.rows[index], field.rows[index-1] = field.rows[index-1], field.rows[index]
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Rows')

    def clear_row_crops(self, row: Row) -> None:
        row.crops = []
        self.field_provider.invalidate()
        self.show_field_settings.refresh()
        self.panels.set_value('Rows')
