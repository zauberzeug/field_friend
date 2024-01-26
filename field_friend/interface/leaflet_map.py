
import logging
from typing import TYPE_CHECKING
import uuid
from nicegui import events, ui, elements
import fiona
import rosys
from ..automations import FieldProvider, Field
from geographiclib.geodesic import Geodesic
from copy import deepcopy
from field_friend.navigation.point_transformation import wgs84_to_cartesian, cartesian_to_wgs84
import numpy as np
# Enable fiona driver
fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
fiona.drvsupport.supported_drivers['KML'] = 'rw'


if TYPE_CHECKING:
    from field_friend.system import System


class leaflet_map:
    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('field_friend.leaflet_map')
        self.system = system
        self.field_provider = system.field_provider
        self.gnss = system.gnss
        self.draw_control = {
            'draw': {
                'polygon': True,
                'marker': True,
                'circle': False,
                'rectangle': False,
                'polyline': False,
                'circlemarker': False,
            },
            'edit': False,
        }
        self.m = ui.leaflet(center=(54.593578543, 8.94665825599984), zoom=13,
                            draw_control=self.draw_control)
        self.marker = None
        # self.roboter_reference_id = ""

        def set_simulated_reference(latlon, dialog):
            self.gnss.clear_reference()
            self.gnss.set_reference(latlon[0], latlon[1])
            dialog.close()
            ui.notify(f'Robot reference has been set to {latlon[0]}, {latlon[1]}')

        def handle_draw(e: events.GenericEventArguments):
            if e.args['layerType'] == 'marker':
                latlon = (e.args['layer']['_latlng']['lat'],
                          e.args['layer']['_latlng']['lng'])
                if not self.marker:
                    self.marker = self.m.marker(latlng=latlon)
                else:
                    self.marker.move(latlon[0], latlon[1])
                    icon = 'L.icon({iconUrl: "http://leafletjs.com/examples/custom-icons/leaf-green.png"})'
                    self.marker.run_method(':setIcon', icon)

                if system.is_real:
                    print("is real")
                else:
                    with ui.dialog() as simulated_marker_dialog, ui.card():
                        ui.label('You are currently working in a simulation. What does the placed point represent?')
                        ui.button('simulated roboter reference point',
                                  on_click=lambda: set_simulated_reference(latlon, simulated_marker_dialog))
                        ui.button('a boundary point', on_click=simulated_marker_dialog.close)
                        ui.button('Close', on_click=simulated_marker_dialog.close)
                    simulated_marker_dialog.open()
            if e.args['layerType'] == 'polygon':
                coordinates = e.args['layer']['_latlngs']
                coordinates_cart = []
                reference_point = [coordinates[0][0]['lat'], coordinates[0][0]['lng']]
                for point in coordinates[0]:
                    # calc the cartesian coordiantes
                    cart_coords = wgs84_to_cartesian(reference_point, point)
                    current_point = rosys.geometry.Point(x=cart_coords[0], y=cart_coords[1])
                    coordinates_cart.append(current_point)
                field = Field(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}', outline=coordinates_cart,
                              outline_wgs84=coordinates)
                self.field_provider.add_field(field)

        with self.m as m:
            m.on('draw:created', handle_draw)
            self.update_layers()
            self.field_provider.FIELDS_CHANGED.register(self.update_layers)
        self.gnss.ROBOT_LOCATED.register(self.update_robot_position)

    def update_layers(self) -> None:
        self.m.clear_layers()
        self.m.tile_layer(
            url_template=r'https://tile.openstreetmap.de/{z}/{x}/{y}.png',
            options={
                'maxZoom': 18,
                'attribution':
                    '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            },
        )
        for field in self.field_provider.fields:
            self.m.generic_layer(name="polygon", args=[field.outline_wgs84])
            if len(field.outline_wgs84) > 0:
                self.m.set_center((field.outline_wgs84[0][0], field.outline_wgs84[0][1]))
        self.marker = None

    def update_robot_position(self, pose: rosys.geometry.Pose) -> None:
        ref = [self.gnss.reference_lat, self.gnss.reference_lon]
        wgs84_coords = cartesian_to_wgs84(ref, [pose.x, pose.y])
        if not self.marker:
            self.marker = self.m.marker(latlng=(self.gnss.reference_lat, self.gnss.reference_lon))
        self.marker.move(wgs84_coords[0], wgs84_coords[1])
