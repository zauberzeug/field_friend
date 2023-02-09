import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments
from rosys.geometry import Point, Point3d
from rosys.vision import Calibration, Camera, CameraProvider


@dataclass
class CalibrationPoint:
    name: str
    world_position: Point3d
    image_position: Optional[Point] = None

    @staticmethod
    def create(name: str, x: float, y: float, z: float):
        '''Create a calibration point with the given name and world position.'''

        return CalibrationPoint(name=name, world_position=Point3d(x=x, y=y, z=z), image_position=None)

    def svg_position(self, max_x: float, max_y: float) -> str:
        x, y = self.map_image_position(max_x, max_y)
        color = 'red'
        content = f"<line x1='{x}' y1='{y}' x2='{x - 14}' y2='{y}' stroke='{color}' stroke-width='1' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x + 14}' y2='{y}' stroke='{color}' stroke-width='1' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x}' y2='{y - 14}' stroke='{color}' stroke-width='1' />"
        content += f"<line x1='{x}' y1='{y}' x2='{x}' y2='{y + 14}' stroke='{color}' stroke-width='1' />"
        return content

    def svg_text(self, max_x: float, max_y: float) -> str:
        x, y = self.map_image_position(max_x, max_y)
        color = 'red'
        return f"<text x='{x - 15}' y='{y + 15}' stroke='{color}' fill='{color}' font-size='10' direction='rtl'>{self.name}</text>"

    def map_image_position(self, max_x, max_y):
        x = self.image_position.x
        y = self.image_position.y
        if x < 0:
            x = 20
        if y < 0:
            y = 20
        if x > max_x:
            x = max_x - 20
        if y > max_y:
            y = max_y - 20
        return x, y


def create_calibration_pattern():
    return [
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

        # CalibrationPoint.create('X', 0.45, 0, 0),
        # CalibrationPoint.create('X-Top', 0.45, 0, 0.051),

        # CalibrationPoint.create('J', 0.50, 0.10, 0),
        # CalibrationPoint.create('K', 0.50, -0.10, 0),
    ]


class calibration_dialog(ui.dialog):

    def __init__(self, camera_provider: CameraProvider) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.calibration')
        self.camera_provider = camera_provider
        self.points: list[CalibrationPoint] = []
        self.active_point: Point = None
        with self, ui.card().tight().style('max-width: 1000px'):
            events = ['mousemove', 'mousedown', 'mouseup']
            self.calibration_image = \
                ui.interactive_image('', on_mouse=self.on_mouse_move, events=events, cross=False)
            self.calibration_image.style('width: 900px')
            with ui.row().classes('m-4 justify-end items-baseline'):
                self.focal_length_input = ui.number('Focal length')
                ui.button('Apply', on_click=self.apply_calibration)

    async def edit(self, camera: Camera) -> bool:
        self.log.info(f'{camera.id}')
        self.image = camera.latest_captured_image
        if self.image is None:
            return
        self.points = create_calibration_pattern()
        if camera.calibration:
            world_points = np.array([p.world_position.tuple for p in self.points])
            image_points = camera.calibration.project_array_to_image(world_points=world_points)
            for i, point in enumerate(self.points):
                point.image_position = Point(x=image_points[i][0], y=image_points[i][1])
        update = self.calibration_image.set_source(self.camera_provider.get_latest_image_url(camera))
        rosys.task_logger.create_task(update)
        for point in self.points:
            if point.image_position is None:
                point.image_position = Point(x=self.image.size.width/2, y=self.image.size.height/2)
        self.draw_points()
        if camera.focal_length is None:
            camera.focal_length = 400
        self.focal_length_input.value = camera.focal_length
        self.open()
        return (await self) or False

    def draw_points(self) -> None:
        svg = ''
        for point in self.points:
            svg += point.svg_position(max_x=self.image.size.width, max_y=self.image.size.height)
            if not any([p.image_position.distance(point.image_position) < 20 for p in self.points if p != point]):
                svg += point.svg_text(max_x=self.image.size.width, max_y=self.image.size.height)
        self.calibration_image.svg_content = svg

    def on_mouse_move(self, e: MouseEventArguments) -> None:
        if e.type == 'mouseup':
            self.active_point = None
            self.draw_points()
        if e.type == 'mousedown' or (e.type == 'mousemove' and self.active_point):
            self.active_point = self.closest_point(e.image_x, e.image_y)
            self.active_point.image_position.x = e.image_x
            self.active_point.image_position.y = e.image_y
            self.draw_points()

    def closest_point(self, x: float, y: float) -> CalibrationPoint:
        return sorted(self.points, key=lambda p: p.image_position.distance(Point(x=x, y=y)))[0]

    def apply_calibration(self) -> None:
        camera = self.camera_provider.cameras[self.image.camera_id]
        world_points = [p.world_position for p in self.points]
        image_points = [p.image_position for p in self.points]
        f0 = self.focal_length_input.value
        try:
            camera.calibration = Calibration.\
                from_points(world_points=world_points, image_points=image_points, image_size=self.image.size, f0=f0)
        except BaseException as err:
            camera.calibration = None
            ui.notify(str(err))
        else:
            ui.notify('Calibration applied')
            self.camera_provider.needs_backup = True
            self.submit(True)
