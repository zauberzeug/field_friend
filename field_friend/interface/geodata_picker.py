import platform
from pathlib import Path
from typing import Optional
import json
import geopandas as gpd
import xml.etree.ElementTree as ET
import rosys
from shapely.ops import transform
import uuid
from ..automations import Field, FieldProvider
import fiona
import tempfile
from nicegui import events, ui
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
                    "Only a single polygon will be processed. Supported file formates: .xml with ISO 11783, .shp, .kml ").classes('w-80')
            with ui.row():
                # TODO multiple false wäre  schöner. Wenn shp load aber nicht anders funktioniert dann bleibt das so
                ui.upload(on_upload=self.restore_from_file, multiple=True)
            with ui.row().classes('w-full justify-end'):
                ui.button('Cancel', on_click=self.close).props('outline')

    def extract_coordinates_kml(self, event: events.UploadEventArguments) -> list:
        try:
            coordinates = []
            gdf = gpd.read_file(event.content, drivr="KML")
            x_coordinate, y_coordinate = gdf['geometry'].iloc[0].xy
            extracted_points = list(zip(x_coordinate, y_coordinate))
            for point in extracted_points:
                coordinates.append([point[1], point[0]])
            return coordinates
        except:
            rosys.notify("shape file uploads is currently not working. Work in progress.")
            return None

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

    def extract_coordinates_shp(self, event: events.UploadEventArguments) -> list:
        # FIXME does not work. Problem might be that only loading the shp file causes the problem and the programm has no access to shx and dbf
        coordinates = []
        gdf = gpd.read_file(event.content)
        gdf['geometry'] = gdf['geometry'].apply(lambda geom: transform(self.swap_coordinates, geom))
        feature = json.loads(gdf.to_json())
        coordinates = feature["features"][0]["geometry"]["coordinates"][0]
        return coordinates

    def swap_coordinates(self, lon, lat):
        return lat, lon

    async def restore_from_file(self, e: events.UploadEventArguments) -> None:
        print(e.name)
        self.close()
        coordinates = []
        if e is None or e.content is None:
            rosys.notify("You can only upload the following file formates: .shp, .kml and .xml.")
            return
        elif e.name[-3:].casefold() == "shp":
            coordinates = self.extract_coordinates_shp(e)
        elif e.name[-3:].casefold() == "kml":
            coordinates = self.extract_coordinates_kml(e)
        elif e.name[-3:].casefold() == "xml":
            coordinates = self.extract_coordinates_xml(e)
        else:
            rosys.notify("You can only upload the following file formates: .shp, .kml and .xml.")
            return
        if len(coordinates) > 1 and coordinates[0] == coordinates[-1]:
            coordinates.pop()
        reference_point = coordinates[0]
        if coordinates is None:
            rosys.notify("An error occurred while importing the file")
            return
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'{new_id}', outline_wgs84=coordinates,
                      reference_lat=reference_point[0], reference_lon=reference_point[1])
        print(field)
        self.field_provider.add_field(field)
        return
