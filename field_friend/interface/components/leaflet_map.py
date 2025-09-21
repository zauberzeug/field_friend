from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from nicegui import app, ui
from nicegui.elements.leaflet_layers import GenericLayer, Marker, TileLayer
from rosys.geometry import GeoPoint

if TYPE_CHECKING:
    from ...system import System


class LeafletMap:
    def __init__(self, system: System, draw_tools: bool) -> None:
        self.log = logging.getLogger('field_friend.leaflet_map')
        self.system = system
        self.field_provider = system.field_provider
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
        center_point = GeoPoint.from_degrees(51.983159, 7.434212)
        self.m: ui.leaflet
        if draw_tools:
            self.m = ui.leaflet(center=center_point.degree_tuple, zoom=13, draw_control=self.draw_control)
        else:
            self.m = ui.leaflet(center=center_point.degree_tuple, zoom=13)
        self.m.clear_layers()
        self.current_basemap: TileLayer | None = None
        self.toggle_basemap()
        self.field_layers: list[GenericLayer] = []
        self.robot_marker: Marker | None = None
        self.drawn_marker = None
        self.row_layers: list = []
        self.field_provider.FIELDS_CHANGED.register_ui(self.update_layers)
        self.field_provider.FIELD_SELECTED.register_ui(self.update_layers)
        self.system.GNSS_REFERENCE_CHANGED.register_ui(self.update_layers)
        self.update_layers()
        self.update_robot_position()
        self.zoom_to_robot()
        ui.timer(5, self.update_robot_position)

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
        ui.button(icon='polyline', on_click=self.zoom_to_field).props('dense flat') \
            .tooltip('center map on field boundaries').classes('ml-0')
        ui.button('Update reference', on_click=self.system.update_gnss_reference).props('outline color=warning') \
            .tooltip('Set current position as geo reference and restart the system').classes('ml-auto')

    def abort_point_drawing(self, dialog) -> None:
        self.on_dialog_close()
        dialog.close()

    def update_layers(self) -> None:
        for layer in self.field_layers:
            if layer in self.m.layers:
                self.m.remove_layer(layer)
        self.field_layers = []
        for field in self.field_provider.fields:
            color = '#6E93D6' if self.field_provider.selected_field is not None and field.id == self.field_provider.selected_field.id else '#999'
            field_outline = [p.degree_tuple for p in field.outline]
            self.field_layers.append(self.m.generic_layer(name='polygon', args=[field_outline, {'color': color}]))
        for layer in self.row_layers:
            self.m.remove_layer(layer)
        self.row_layers = []
        if self.field_provider.selected_field is not None:
            for row in self.field_provider.selected_field.rows:
                row_points = [p.degree_tuple for p in row.points]
                self.row_layers.append(self.m.generic_layer(name='polyline', args=[row_points, {'color': '#F2C037'}]))

    def update_robot_position(self, dialog=None) -> None:
        # TODO: where does the dialog come from?
        if dialog:
            self.on_dialog_close()
            dialog.close()
        geo_point = GeoPoint.from_point(self.system.robot_locator.pose.point)
        self.log.debug('Updating robot position: %s', geo_point)
        self.robot_marker = self.robot_marker or self.m.marker(latlng=geo_point.degree_tuple)
        icon = 'L.icon({iconUrl: "assets/robot_position_side.png", iconSize: [50,50], iconAnchor:[20,20]})'
        self.robot_marker.run_method(':setIcon', icon)
        self.robot_marker.move(*geo_point.degree_tuple)

    def zoom_to_robot(self) -> None:
        geo_point = GeoPoint.from_point(self.system.robot_locator.pose.point)
        self.log.debug('Zooming to robot position: %s', geo_point)
        self.m.set_center(geo_point.degree_tuple)
        if self.current_basemap is not None:
            self.m.set_zoom(self.current_basemap.options['maxZoom'] - 1)

    def zoom_to_field(self) -> None:
        field = self.field_provider.selected_field if self.field_provider.selected_field else None
        if field is None:
            return
        coords = [geo_point.degree_tuple for geo_point in field.outline]
        center = sum(lat for lat, _ in coords) / len(coords), sum(lon for _, lon in coords) / len(coords)
        self.log.debug('Zooming to field boundaries: %s', center)
        self.m.set_center(center)
        if self.current_basemap is not None:
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

    def on_dialog_close(self) -> None:
        if self.drawn_marker is not None:
            self.m.remove_layer(self.drawn_marker)
        self.drawn_marker = None
