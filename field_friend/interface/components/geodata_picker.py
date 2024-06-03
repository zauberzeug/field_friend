import json
import platform
import tempfile
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional

import fiona
import geopandas as gpd
import rosys
from nicegui import events, ui
from shapely.ops import transform

from ...automations import Field, FieldProvider

# Enable fiona driver
fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
fiona.drvsupport.supported_drivers['KML'] = 'rw'
fiona.drvsupport.supported_drivers['LIBKML'] = 'rw'


class geodata_picker(ui.dialog):
    def __init__(self, field_provider: FieldProvider) -> None:
        super().__init__()
        self.field_provider = field_provider
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
            with ui.row().classes('w-full justify-end'):
                ui.button('Cancel', on_click=self.close).props('outline')

    def extract_coordinates_kml(self, event: events.UploadEventArguments) -> list:
        coordinates = []
        gdf = gpd.read_file(event.content, drivr="KML")
        x_coordinate, y_coordinate = gdf['geometry'].iloc[0].xy
        extracted_points = list(zip(x_coordinate, y_coordinate))
        for point in extracted_points:
            coordinates.append([point[1], point[0]])
        return coordinates

    def extract_coordinates_xml(self, event: events.UploadEventArguments) -> list:
        coordinates = []
        tree = ET.parse(event.content)
        root = tree.getroot()
        for geo_data in root.findall('.//LSG'):
            for point in geo_data.findall('.//PNT'):
                lat = float(point.attrib['C'])
                lon = float(point.attrib['D'])
                coordinates.append([lat, lon])
        return coordinates

    def extract_coordinates_shp(self, event: events.UploadEventArguments) -> Optional[list]:
        coordinates = []
        print(event.content)
        try:
            gdf = gpd.read_file(event.content)
            print(gdf)
            gdf['geometry'] = gdf['geometry'].apply(lambda geom: transform(self.swap_coordinates, geom))
            feature = json.loads(gdf.to_json())
            coordinates = feature["features"][0]["geometry"]["coordinates"][0]
            return coordinates
        except:
            rosys.notify("The .zip file does not contain a shape file.", type='warning')
            return None

    def swap_coordinates(self, lon, lat):
        return lat, lon

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
        if len(coordinates) > 1 and coordinates[0] == coordinates[-1]:
            coordinates.pop()
        self.field_provider.create_field(points=coordinates)
        return
