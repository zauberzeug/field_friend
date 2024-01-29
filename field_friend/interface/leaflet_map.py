
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
        self.field_layers: list[list] = []
        self.robot_marker = None
        self.drawn_marker = None

        def handle_draw(e: events.GenericEventArguments):
            if e.args['layerType'] == 'marker':
                latlon = (e.args['layer']['_latlng']['lat'],
                          e.args['layer']['_latlng']['lng'])
                self.drawn_marker = self.m.marker(latlng=latlon)
                if system.is_real:
                    # TODO implement  a dialog for  a real system
                    print("is real")
                else:
                    with ui.dialog() as simulated_marker_dialog, ui.card():
                        ui.label('You are currently working in a simulation. What does the placed point represent?')
                        ui.button('simulated roboter reference point',
                                  on_click=lambda: self.set_simulated_reference(latlon, simulated_marker_dialog))
                        ui.button('as point for the current object',
                                  on_click=lambda: self.add_point_active_object(latlon, simulated_marker_dialog))
                        ui.button('Close', on_click=simulated_marker_dialog.close)
                    simulated_marker_dialog.open()
            if e.args['layerType'] == 'polygon':
                coordinates = e.args['layer']['_latlngs']
                point_list = []
                for point in coordinates[0]:
                    point_list.append([point['lat'], point['lng']])
                field = Field(id=f'{str(uuid.uuid4())}', name=f'{str(uuid.uuid4())}',
                              outline_wgs84=point_list)
                self.field_provider.add_field(field)

        with self.m as m:
            m.on('draw:created', handle_draw)
            self.update_layers()
            self.field_provider.FIELDS_CHANGED.register(self.update_layers)
        self.gnss.ROBOT_LOCATED.register(self.update_robot_position)

    def set_simulated_reference(self, latlon, dialog):
        self.gnss.clear_reference()
        self.gnss.set_reference(latlon[0], latlon[1])
        dialog.close()
        self.m.remove_layer(self.drawn_marker)
        ui.notify(f'Robot reference has been set to {latlon[0]}, {latlon[1]}')

    def add_point_active_object(self, latlon, dialog) -> None:
        dialog.close()
        self.m.remove_layer(self.drawn_marker)
        if self.field_provider.active_object is not None and self.field_provider.active_object["object"] is not None:
            print(latlon)
            self.field_provider.active_object["object"].points_wgs84.append([latlon[0], latlon[1]])
            self.field_provider.OBJECT_SELECTED.emit()
        else:
            ui.notify("No object selected. Point could not be added to the void.")

    def update_layers(self) -> None:
        for layer in self.field_layers:
            self.m.remove_layer(layer)
        self.field_layers = []
        for field in self.field_provider.fields:
            self.field_layers.append(self.m.generic_layer(name="polygon", args=[field.outline_wgs84]))

    def update_robot_position(self, pose: rosys.geometry.Pose) -> None:
        reference = self.gnss.get_reference()
        wgs84_coords = cartesian_to_wgs84(reference, [pose.x, pose.y])
        if not self.robot_marker:
            # TODO: use an icon that displays the robot well
            icon = 'L.icon({iconUrl: "http://leafletjs.com/examples/custom-icons/leaf-red.png"})'
            self.robot_marker = self.m.marker(latlng=(wgs84_coords[0], wgs84_coords[1]))
            self.robot_marker.run_method(':setIcon', icon)
        self.robot_marker.move(wgs84_coords[0], wgs84_coords[1])
