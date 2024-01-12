
import logging

from nicegui import events, ui
import fiona
import rosys

# Enable fiona driver
# gpd.io.file.fiona.drvsupport.supported_drivers['KML'] = 'rw'
fiona.drvsupport.supported_drivers['kml'] = 'rw'  # enable KML support which is disabled by default
fiona.drvsupport.supported_drivers['KML'] = 'rw'


class leaflet_map:

    def __init__(self) -> None:
        self.log = logging.getLogger('field_friend.map')

        # TODO: make the leaflet map listen to changes of the field provider and show then all fields with WGS84 cords as layer
        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

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

    def add_layer(self, layer_type, content) -> None:
        self.m.generic_layer(name=layer_type, args=[content])
