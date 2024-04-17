import json
import math
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import fiona
import geopandas as gpd
from shapely import difference, offset_curve
from shapely.geometry import (GeometryCollection, LinearRing, LineString, MultiLineString, MultiPoint, Point, Polygon,
                              mapping)
from shapely.ops import transform

import rosys
from nicegui import events, ui

from ...automations import Field, FieldObstacle, FieldProvider, Row
from ...navigation.point_transformation import cartesian_to_wgs84

# Enable fiona driver
fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
fiona.drvsupport.supported_drivers['KML'] = 'rw'
fiona.drvsupport.supported_drivers['LIBKML'] = 'rw'


class geodata_picker(ui.dialog):
    def __init__(self, field_provider: FieldProvider) -> None:
        super().__init__()
        self.field_provider = field_provider
        self.is_farmdroid = False
        self.safety_distance = 2.7
        self.working_width = 2.7
        self.headland_tracks = 1
        self.ab_line = 1

        with self, ui.card():
            with ui.row():
                ui.label("Upload a file.").classes('text-xl w-80')
            with ui.row():
                ui.label(
                    "Only a single polygon will be processed. Supported file formates: .xml with ISO 11783, .shp, .kml.").classes('w-80')
            with ui.row():
                ui.label(
                    "If you want to upload a shape,  create a zip-file containing all files  (minimum: .shp, .shx, .dbf) and upload the zip.").classes('w-80')
            with ui.row():
                ui.upload(on_upload=self.restore_from_file, multiple=False)
            with ui.row():
                farmdroid_checkbox = ui.checkbox('FarmDroid Data', on_change=lambda e: self.set_farmdroid(e.value))
            with ui.row().bind_visibility_from(farmdroid_checkbox, 'value').classes('w-80'):
                with ui.row().classes('w-full place-items-center'):
                    with ui.icon('help').classes('text-xl'):
                        ui.tooltip('The width of the attached tool.')
                    ui.number(label='Working Width (in m)', value=self.working_width, format='%.2f',
                              on_change=lambda e: self.set_working_width(e.value))
                with ui.row().classes('w-full place-items-center'):
                    with ui.icon('help').classes('text-xl'):
                        ui.tooltip('The distance between the border of the field and the headland.')
                    ui.number(label='Safety Distance (in m)', value=self.safety_distance, format='%.2f',
                              on_change=lambda e: self.set_safety_distance(e.value))
                with ui.row().classes('w-full place-items-center'):
                    with ui.icon('help').classes('text-xl'):
                        ui.tooltip('Number of headland tracks.')
                    ui.number(label='headland Tracks', value=self.headland_tracks, format='%.2f',
                              on_change=lambda e: self.set_headland_tracks(e.value))
                with ui.row().classes('w-full place-items-center'):
                    with ui.icon('help').classes('text-xl'):
                        ui.tooltip('In which direction does the AB-line run?')
                    switch = ui.switch('AB-Line Direction', on_change=lambda e: self.set_ab_line(e.value))
                # TODO EINGABE FÜR DAS OFFSET DER FARMDROID DATEN

            with ui.row().classes('w-full justify-end'):
                ui.button('Cancel', on_click=self.close).props('outline')

    def set_farmdroid(self, value) -> None:
        self.is_farmdroid = value

    def set_ab_line(self, value) -> None:
        if value:
            self.ab_line = 2
        else:
            self.ab_line = 1

    def set_safety_distance(self, value) -> None:
        self.safety_distance = value

    def set_working_width(self, value) -> None:
        self.working_width = value

    def set_headland_tracks(self, value) -> None:
        self.headland_tracks = value

    def extract_coordinates_kml(self, event: events.UploadEventArguments) -> list:
        coordinates = []
        gdf = gpd.read_file(event.content, drivr="KML")
        x_coordinate, y_coordinate = gdf['geometry'].iloc[0].xy
        extracted_points = list(zip(x_coordinate, y_coordinate))
        for point in extracted_points:
            coordinates.append([point[1], point[0]])
        return [coordinates]

    def extract_coordinates_xml(self, event: events.UploadEventArguments) -> list:
        coordinates = []
        tree = ET.parse(event.content)
        root = tree.getroot()
        for geo_data in root.findall('.//LSG'):
            for point in geo_data.findall('.//PNT'):
                lat = float(point.attrib['C'])
                lon = float(point.attrib['D'])
                coordinates.append([lat, lon])
        return [coordinates]

    def extract_coordinates_shp(self, event: events.UploadEventArguments) -> Optional[list]:
        coordinates = []
        try:
            gdf = gpd.read_file(event.content)
            gdf['geometry'] = gdf['geometry'].apply(lambda geom: transform(self.swap_coordinates, geom))
            feature = json.loads(gdf.to_json())
            coordinates = feature["features"][0]["geometry"]["coordinates"]
            return coordinates
        except:
            rosys.notify("The .zip file does not contain a shape file.", type='warning')
            return None

    def swap_coordinates(self, lon, lat):
        return lat, lon

    def get_extrapoled_line(self, p1, p2) -> LineString:
        'Creates a line with two points extrapoled in both direction'
        EXTRAPOL_RATIO = 10
        a = [p1[0]+EXTRAPOL_RATIO*(p1[0]-p2[0]), p1[1]+EXTRAPOL_RATIO*(p1[1]-p2[1])]
        b = [p2[0]+EXTRAPOL_RATIO*(p2[0]-p1[0]), p2[1]+EXTRAPOL_RATIO*(p2[1]-p1[1])]
        return LineString([a, b])

    def get_rows(self, field: Field) -> list:
        if not self.is_farmdroid:
            return []
        if self.safety_distance + (self.working_width / 2) < 2.7:
            rosys.notify(
                'The distance between the outer headland track and the field boundary is too small. Please check the specified values.')
            return []
        lines = []
        outline = []
        for point in field.outline:
            outline.append([point.x, point.y])
        outline_polygon = Polygon(outline)
        buffer_width = self.safety_distance + (self.headland_tracks * self.working_width)
        row_spacing = self.working_width / 5
        working_area_meter = outline_polygon.buffer(-buffer_width, join_style='mitre', mitre_limit=math.inf)
        field.working_area = working_area_meter.area
        working_area_coordinates_meter = mapping(working_area_meter)['coordinates'][0]
        working_area_coordinates = []
        for point in working_area_coordinates_meter:
            transformed_point = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [-point[1], point[0]])
            working_area_coordinates.append([transformed_point[0], transformed_point[1]])
        if self.ab_line == 1:
            ab_line = LineString([(working_area_coordinates[0][0], working_area_coordinates[0][1]),
                                  (working_area_coordinates[1][0], working_area_coordinates[1][1])])
            ab_line_meter = LineString([(working_area_coordinates_meter[0][0], working_area_coordinates_meter[0][1]),
                                        (working_area_coordinates_meter[1][0], working_area_coordinates_meter[1][1])])
        if self.ab_line == 2:
            if working_area_coordinates[0][0] == working_area_coordinates[-1][0] and working_area_coordinates[0][1] == working_area_coordinates[-1][1]:
                ab_line = LineString([(working_area_coordinates[0][0], working_area_coordinates[0][1]),
                                      (working_area_coordinates[-2][0], working_area_coordinates[-2][1])])
                ab_line_meter = LineString([(working_area_coordinates_meter[0][0], working_area_coordinates_meter[0][1]),
                                            (working_area_coordinates_meter[-2][0], working_area_coordinates_meter[-2][1])])
            else:
                ab_line = LineString([(working_area_coordinates[0][0], working_area_coordinates[0][1]),
                                      (working_area_coordinates[-1][0], working_area_coordinates[-1][1])])
                ab_line_meter = LineString([(working_area_coordinates_meter[0][0], working_area_coordinates_meter[0][1]),
                                            (working_area_coordinates_meter[-1][0], working_area_coordinates_meter[-1][1])])

        direction_check = offset_curve(ab_line_meter, -row_spacing)
        if working_area_meter.contains(direction_check) or working_area_meter.intersects(direction_check):
            row_spacing = -row_spacing
        lines_meter = []
        lines_meter.append(ab_line_meter)
        lines.append(ab_line)
        line_inside = True
        while line_inside:
            line_meter = offset_curve(lines_meter[-1], row_spacing)
            if working_area_meter.contains(line_meter) or working_area_meter.intersects(line_meter):
                lines_meter.append(line_meter)
                line = []
                for point in mapping(line_meter)["coordinates"]:
                    point_wgs84 = cartesian_to_wgs84([field.reference_lat, field.reference_lon], [-point[1], point[0]])
                    line.append(point_wgs84)
                linestring = LineString([(line[0][0], line[0][1]), (line[1][0], line[1][1])])
                lines.append(linestring)
            else:
                line_inside = False
        working_area_ring = LinearRing(working_area_coordinates)
        rows = []
        for line in lines:
            point_list = list(mapping(line)['coordinates'])
            for index, point in enumerate(point_list):
                point_list[index] = list(point)
            extrapolated_line = self.get_extrapoled_line(point_list[0], point_list[1])
            intersection_geometry = working_area_ring.intersection(extrapolated_line)
            if isinstance(intersection_geometry, MultiPoint):
                intersection_list = []
                # TODO hier noch Fälle beachten, bei denen die Linie den Umring in mehr als zwei getrennten Punkten/Segmenten schneidet
                for point in list(mapping(intersection_geometry)['coordinates']):
                    intersection_list.append([point[0], point[1]])
                divided_lines = []
                for obstacle in field.obstacles:
                    divided_lines.append(difference(LineString(intersection_list), Polygon(obstacle.points_wgs84)))
                if any(isinstance(x, MultiLineString) for x in divided_lines):
                    for segment in divided_lines:
                        if isinstance(segment, MultiLineString):
                            for line in mapping(segment)['coordinates']:
                                row_coordinates = []
                                for point in line:
                                    row_coordinates.append([point[0], point[1]])
                                row_id = str(uuid.uuid4())
                                new_row = Row(id=f'{row_id}', name=f'{row_id}', points_wgs84=row_coordinates)
                                rows.append(new_row)
                else:
                    row_id = str(uuid.uuid4())
                    new_row = Row(id=f'{row_id}', name=f'{row_id}', points_wgs84=intersection_list)
                    rows.append(new_row)
            elif isinstance(intersection_geometry, GeometryCollection):
                for geometry in mapping(intersection_geometry)['geometries']:
                    if geometry == LineString or geometry == MultiPoint:
                        intersection_list = []
                        # TODO hier noch Fälle beachten, bei denen die Linie den Umring in mehr als zwei getrennten Punkten/Segmenten schneidet
                        for point in list(mapping(geometry)['coordinates']):
                            intersection_list.append([point[0], point[1]])
                        divided_lines = []
                        for obstacle in field.obstacles:
                            divided_lines.append(difference(LineString(intersection_list),
                                                 Polygon(obstacle.points_wgs84)))
                        if any(isinstance(x, MultiLineString) for x in divided_lines):
                            for segment in divided_lines:
                                if isinstance(segment, MultiLineString):
                                    for line in mapping(segment)['coordinates']:
                                        row_coordinates = []
                                        for point in line:
                                            row_coordinates.append([point[0], point[1]])
                                        row_id = str(uuid.uuid4())
                                        new_row = Row(id=f'{row_id}', name=f'{row_id}', points_wgs84=row_coordinates)
                                        rows.append(new_row)
                        else:
                            row_id = str(uuid.uuid4())
                            new_row = Row(id=f'{row_id}', name=f'{row_id}', points_wgs84=intersection_list)
                            rows.append(new_row)
                    # elif type(geometry) == Point:
                    #     intersection_list.append([geometry[0], geometry[1]])
        return rows

    def get_obstacles(self, polygon_list):
        obstacle_list = []
        for polygon in polygon_list[1:]:
            row_id = str(uuid.uuid4())
            new_obstacle = FieldObstacle(id=f'{row_id}', name=f'{row_id}', points_wgs84=polygon)
            obstacle_list.append(new_obstacle)
        return obstacle_list

    async def restore_from_file(self, e: events.UploadEventArguments) -> None:
        self.close()
        coordinates: list = []
        if e is None or e.content is None:
            rosys.notify("You can only upload the following file formates: .kml ,.xml. with ISO  and shape files.", type='warning')
            return
        elif e.name[-3:].casefold() == "zip":
            coordinates = self.extract_coordinates_shp(e)
        elif e.name[-3:].casefold() == "kml":
            coordinates = self.extract_coordinates_kml(e)
        elif e.name[-3:].casefold() == "xml":
            coordinates = self.extract_coordinates_xml(e)
        else:
            rosys.notify("You can only upload the following file formates: .kml ,.xml. with ISO  and shape files.", type='warning')
            return
        if coordinates is None:
            rosys.notify("An error occurred while importing the file.", type='negative')
            return
        reference_point = coordinates[0][0]
        field_id = str(uuid.uuid4())
        field = Field(id=f'{field_id}', name=f'{field_id}', outline_wgs84=coordinates[0],
                      reference_lat=reference_point[0], reference_lon=reference_point[1])
        obstacles = []
        if len(coordinates) > 1:
            obstacles = self.get_obstacles(coordinates)
        field.obstacles = obstacles
        rows = self.get_rows(field)
        field.rows = rows
        outline = []
        for point in field.outline:
            outline.append([point.x, point.y])
        field.area = Polygon(outline).area
        if len(coordinates[0]) > 2 and coordinates[0][0] == coordinates[0][-1]:
            coordinates[0].pop()  # the last point is the same as the first point
        self.field_provider.add_field(field)
        return
