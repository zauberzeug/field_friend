
import logging
import uuid
from typing import TYPE_CHECKING, Any, Literal, TypedDict

import rosys
import rosys.geometry
from nicegui import app, events, ui
from nicegui.elements.leaflet_layers import GenericLayer, TileLayer

from ...automations import Field, FieldObstacle, FieldProvider, Row
from ...localization.geo_point import GeoPoint
from .key_controls import KeyControls


class Active_object(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Row | FieldObstacle


if TYPE_CHECKING:
    from field_friend.system import System


class leaflet_map:
    def __init__(self, system: 'System', draw_tools: bool) -> None:
        self.log = logging.getLogger('field_friend.leaflet_map')
        self.system = system
        self.field_provider = system.field_provider
        self.key_controls = KeyControls(self.system)
        self.draw_tools = draw_tools
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
        center_point = GeoPoint(lat=51.983159, long=7.434212)
        if self.system.gnss.current is not None and self.system.gnss.current.location is not None:
            center_point = self.system.gnss.current.location
        self.m: ui.leaflet
        if draw_tools:
            self.m = ui.leaflet(center=center_point.tuple,
                                zoom=13, draw_control=self.draw_control)
        else:
            self.m = ui.leaflet(center=center_point.tuple,
                                zoom=13)
        self.m.clear_layers()
        self.current_basemap: TileLayer | None = None
        self.toggle_basemap()
        self.field_layers: list[GenericLayer] = []
        self.robot_marker = None
        self.drawn_marker = None
        self.obstacle_layers: list = []
        self.row_layers: list = []
        self._active_field: str | None = None
        self.active_object: Active_object | None = None
        self.update_layers()
        self.field_provider.FIELDS_CHANGED.register(self.update_layers)
        self.zoom_to_robot()

        def handle_draw(e: events.GenericEventArguments):
            if e.args['layerType'] == 'marker':
                latlon = (e.args['layer']['_latlng']['lat'],
                          e.args['layer']['_latlng']['lng'])
                self.drawn_marker = self.m.marker(latlng=latlon)
                if system.is_real:
                    with ui.dialog() as real_marker_dialog, ui.card():
                        ui.label('You are currently working in a real system. What does the placed point represent?')
                        ui.button('add point to selected object (row, obstacle)',
                                  on_click=lambda: self.add_point_active_object(latlon, real_marker_dialog))
                        ui.button('Close', on_click=lambda: self.abort_point_drawing(real_marker_dialog))
                    real_marker_dialog.open()
                else:
                    with ui.dialog() as simulated_marker_dialog, ui.card():
                        ui.label('You are currently working in a simulation. What does the placed point represent?')
                        ui.button('as point for the current object',
                                  on_click=lambda: self.add_point_active_object(latlon, simulated_marker_dialog))
                        ui.button('Close', on_click=lambda: self.abort_point_drawing(simulated_marker_dialog))
                    simulated_marker_dialog.open()
            if e.args['layerType'] == 'polygon':
                coordinates = e.args['layer']['_latlngs']
                point_list = []
                for point in coordinates[0]:
                    point_list.append(GeoPoint(lat=point['lat'], long=point['lng']))
                self.field_provider.create_field(points=point_list)

        with self.m as m:
            m.on('draw:created', handle_draw)
        self.gnss.ROBOT_GNSS_POSITION_CHANGED.register_ui(self.update_robot_position)

    def buttons(self) -> None:
        """Builds additional buttons to interact with the map."""
        ui.button(icon='satellite', on_click=self.toggle_basemap).props('dense flat') \
            .bind_visibility_from(self, 'current_basemap', lambda x: x is not None and 'openstreetmap' not in x.url_template) \
            .tooltip('Switch to map view')
        ui.button(icon='map', on_click=self.toggle_basemap).props('dense flat') \
            .bind_visibility_from(self, 'current_basemap', lambda x: x is not None and 'openstreetmap' in x.url_template) \
            .tooltip('Switch to satellite view')
        ui.button(icon='my_location', on_click=self.zoom_to_robot).props('dense flat') \
            .tooltip('Center map on robot position').classes('ml-0')
        ui.button(on_click=self.zoom_to_field) \
            .bind_enabled_from(self, 'active_field') \
            .props('icon=polyline dense flat') \
            .tooltip('center map on field boundaries').classes('ml-0')

    def abort_point_drawing(self, dialog) -> None:
        if self.drawn_marker is not None:
            self.m.remove_layer(self.drawn_marker)
        self.drawn_marker = None
        dialog.close()

    def add_point_active_object(self, latlon, dialog) -> None:
        dialog.close()
        self.m.remove_layer(self.drawn_marker)
        if self.active_object is not None and self.active_object["object"] is not None:
            self.active_object["object"].points.append(GeoPoint.from_list(latlon))
            self.field_provider.invalidate()
            self.update_layers()
        else:
            ui.notify("No object selected. Point could not be added to the void.")

    def update_layers(self) -> None:
        for layer in self.field_layers:
            if layer in self.m.layers:
                self.m.remove_layer(layer)
        self.field_layers = []
        for field in self.field_provider.fields:
            color = '#6E93D6' if field.id == self.active_field else '#999'
            self.field_layers.append(self.m.generic_layer(name="polygon",
                                                          args=[field.points_as_tuples, {'color': color}]))
        field = self.field_provider.get_field(self.active_field)
        if field is None:
            return
        for layer in self.obstacle_layers:
            self.m.remove_layer(layer)
        self.obstacle_layers = []
        for layer in self.row_layers:
            self.m.remove_layer(layer)
        self.row_layers = []
        for obstacle in field.obstacles:
            self.obstacle_layers.append(self.m.generic_layer(name="polygon",
                                                             args=[obstacle.points_as_tuples, {'color': '#C10015'}]))
        for row in field.rows:
            self.row_layers.append(self.m.generic_layer(name="polyline",
                                                        args=[row.points_as_tuples, {'color': '#F2C037'}]))

    @property
    def active_field(self) -> str | None:
        return self._active_field

    @active_field.setter
    def active_field(self, field_id: str | None) -> None:
        self._active_field = field_id
        self.update_layers()

    def update_robot_position(self, position: GeoPoint) -> None:
        if self.robot_marker is None:
            self.robot_marker = self.m.marker(latlng=position.tuple)
        icon = 'L.icon({iconUrl: "assets/robot_position_side.png", iconSize: [50,50], iconAnchor:[20,20]})'
        self.robot_marker.run_method(':setIcon', icon)
        self.robot_marker.move(*position.tuple)

    def zoom_to_robot(self) -> None:
        if self.gnss.current is None:
            self.log.warning('No GNSS position available, could not zoom to robot')
            return
        self.m.set_center(self.gnss.current.location.tuple)
        self.m.set_zoom(self.current_basemap.options['maxZoom'] - 1)

    def zoom_to_field(self) -> None:
        field = self.field_provider.get_field(self.active_field)
        if field is None:
            return
        coords = field.points_as_tuples
        center = sum(lat for lat, _ in coords) / len(coords), sum(lon for _, lon in coords) / len(coords)
        self.m.set_center(center)
        self.m.set_zoom(self.current_basemap.options['maxZoom'] - 1)  # TODO use field boundaries to calculate zoom

    def toggle_basemap(self) -> None:
        use_satellite = app.storage.user.get('use_satellite', False)
        if self.current_basemap is not None:
            self.m.remove_layer(self.current_basemap)
            use_satellite = not use_satellite
            app.storage.user['use_satellite'] = use_satellite
        if use_satellite:
            # ESRI satellite image provides free usage
            self.current_basemap = self.m.tile_layer(
                url_template='https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
                options={
                    'maxZoom': 21,
                    'attribution': 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
                })
            # self.current_basemap = self.m.tile_layer(
            #     url_template=r'http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
            #     options={
            #         'maxZoom': 20,
            #         'subdomains': ['mt0', 'mt1', 'mt2', 'mt3'],
            #         'attribution': '&copy; <a href="https://maps.google.com">Google Maps</a>'
            #     },
            # )
        else:
            self.current_basemap = self.m.tile_layer(
                url_template=r'https://tile.openstreetmap.org/{z}/{x}/{y}.png',
                options={
                    'maxZoom': 20,
                    'attribution': '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
                },
            )
        if self.current_basemap.options['maxZoom'] - 1 < self.m.zoom:
            self.m.set_zoom(self.current_basemap.options['maxZoom'] - 1)
