from dataclasses import dataclass
from typing import Optional

import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments


@dataclass
class CalibrationPoint:
    name: str
    world_position: rosys.geometry.Point3d
    image_position: Optional[rosys.geometry.Point] = None

    @staticmethod
    def create(name: str, x: float, y: float, z: float):
        '''Create a calibration point with the given name and world position.'''

        return CalibrationPoint(name=name, world_position=rosys.geometry.Point3d(x=x, y=y, z=z), image_position=None)

    def svg(self) -> str:
        x = self.image_position.x
        y = self.image_position.y
        color = 'red'
        content = f"<line x1='{x}' y1='{y}' x2='{x - 14}' y2='{y}' stroke='{color}' stroke-width='3' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x + 14}' y2='{y}' stroke='{color}' stroke-width='3' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x}' y2='{y - 14}' stroke='{color}' stroke-width='3' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x}' y2='{y + 14}' stroke='{color}' stroke-width='3' />"
        content += f"<text x='{x - 15}' y='{y + 15}' stroke='{color}' fill='{color}' font-size='20' direction='rtl'>{self.name}</text>"
        return content


calibration_pattern = [
    CalibrationPoint.create('A', 0.20, 0.10, 0),
    CalibrationPoint.create('B', 0.20, 0, 0),
    CalibrationPoint.create('B-Top', 0.20, 0, 0.048),
    CalibrationPoint.create('C', 0.20, -0.10, 0),

    CalibrationPoint.create('D', 0.30, 0.10, 0),
    CalibrationPoint.create('E', 0.30, 0, 0),
    CalibrationPoint.create('F', 0.30, -0.10, 0),

    CalibrationPoint.create('G', 0.40, 0.10, 0),
    CalibrationPoint.create('H', 0.40, 0, 0),
    CalibrationPoint.create('I', 0.40, -0.10, 0),

    CalibrationPoint.create('X', 0.45, 0, 0),
    CalibrationPoint.create('X-Top', 0.45, 0, 0.051),

    CalibrationPoint.create('J', 0.50, 0.10, 0),
    CalibrationPoint.create('K', 0.50, -0.10, 0),
]


class calibration_dialog(ui.dialog):

    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        super().__init__()
        self.camera_provider = camera_provider
        self.active_point = None
        with self, ui.card().tight().style('max-width: 1000px'):
            ui.label('Calibration')
            events = ['mousemove', 'mousedown', 'mouseup']
            self.calibration_image = \
                ui.interactive_image('', on_mouse=self.on_mouse_move, events=events, cross=True)
            self.calibration_image.style('width: 1000px')

    def calibrate(self, camera: rosys.vision.Camera) -> None:
        update = self.calibration_image.set_source(self.camera_provider.get_latest_image_url(camera))
        rosys.task_logger.create_task(update)
        self.open()
        if camera.calibration:
            # TODO project calibration points to image
            pass
        image = camera.latest_captured_image
        if image is None:
            return
        for point in calibration_pattern:
            if point.image_position is None:
                point.image_position = rosys.geometry.Point(x=image.size.width/2, y=image.size.height/2)
        self.draw_points()

    def draw_points(self):
        svg = ''
        for point in calibration_pattern:
            svg += point.svg()
        self.calibration_image.svg_content = svg

    def on_mouse_move(self, e: MouseEventArguments) -> None:
        if e.type == 'mousedown':
            self.active_point = self.closest_point(e.image_x, e.image_y)
        if e.type == 'mouseup':
            self.active_point = None
        if e.type == 'mousemove' and self.active_point:
            self.active_point.image_position.x = e.image_x
            self.active_point.image_position.y = e.image_y
            self.draw_points()

    def closest_point(self, x: float, y: float) -> CalibrationPoint:
        return sorted(calibration_pattern, key=lambda p: p.image_position.distance(rosys.geometry.Point(x=x, y=y)))[0]
