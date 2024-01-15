
import logging

from nicegui import events, ui
import fiona
import rosys
from ..automations import FieldProvider, Field

# Enable fiona driver
# gpd.io.file.fiona.drvsupport.supported_drivers['KML'] = 'rw'
fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
fiona.drvsupport.supported_drivers['KML'] = 'rw'


class leaflet_map:

    def __init__(self, field_provider: FieldProvider) -> None:
        self.log = logging.getLogger('field_friend.leaflet_map')
        self.field_provider = field_provider
        self.draw_control = {
            'draw': {
                'polygon': True,
                'marker': False,
                'circle': False,
                'rectangle': False,
                'polyline': False,
                'circlemarker': False,
            },
            'edit': False,
        }
        self.m = ui.leaflet(center=(54.593578543, 8.94665825599984), zoom=13, draw_control=self.draw_control)

        def handle_draw(e: events.GenericEventArguments):
            if e.args['layerType'] == 'marker':
                self.m.marker(latlng=(e.args['layer']['_latlng']['lat'],
                                      e.args['layer']['_latlng']['lng']))
            if e.args['layerType'] == 'polygon':
                self.m.generic_layer(name='polygon', args=[e.args['layer']['_latlngs']])

        with self.m as m:
            m.on('draw:created', handle_draw)
            self.update_layers()

    @ui.refreshable
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
            self.m.generic_layer(name="polygon", args=[field.outline_wgs84], )
            self.m.set_center((field.outline_wgs84[0][0], field.outline_wgs84[0][1]))
