import abc
import logging
from dataclasses import dataclass

from nicegui import ui
from rosys.event import Event
from rosys.geometry import Point


@dataclass
class Scan:
    time: float
    points: list[Point]


class LaserScanner(abc.ABC):

    def __init__(self) -> None:
        self.log = logging.getLogger('laser_scanner')
        self.last_scan: Scan | None = None
        self.is_active: bool = False

        self.svg_size = (360, 360)
        self.svg_center = (self.svg_size[0] / 2, self.svg_size[1] / 2)
        self.svg_scale = 40.0
        self._map_content = ''
        self.base_svg = f'<svg viewBox="0 0 {self.svg_size[0]} {self.svg_size[1]}" xmlns="http://www.w3.org/2000/svg">'
        self.base_svg += f'<circle cx="{self.svg_center[0]}" cy="{self.svg_center[1]}" \
            r="{3}" fill="red"/>'
        self.last_draw_timestamp: float = 0.0

        self.NEW_SCAN = Event()
        '''a new scan is available (argument: ndarray with scan points in world coordinate system)'''

    def ui(self):
        ui.interactive_image(size=self.svg_size) \
            .bind_content_from(self, '_map_content') \
            .classes(f'w-[{self.svg_size[0]}px] h-[{self.svg_size[1]}px] bg-gray-100')
        ui.timer(0.5, self.draw_last_scan)

    def draw_last_scan(self):
        if self.last_scan is None:
            return
        if self.last_draw_timestamp >= self.last_scan.time:
            return
        self.draw(self.last_scan)
        self.last_draw_timestamp = self.last_scan.time

    def draw(self, scan: Scan):
        svg = self.base_svg
        for point in scan.points:
            x = int(self.svg_center[0] + self.svg_scale * point.x)
            y = int(self.svg_center[1] + self.svg_scale * point.y)
            svg += f'<circle cx="{x}" cy="{y}" r="1" fill="blue"/>'
        svg += '</svg>'
        self._map_content = svg
