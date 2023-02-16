import colorsys
import logging

import numpy as np
import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments, ValueChangeEventArguments
from rosys.automation import Automator
from rosys.geometry import Point, Point3d
from rosys.vision import Camera, CameraProvider, Detector

from ..automations import Puncher
from ..vision import CameraSelector
from .calibration_dialog import calibration_dialog


class camera:

    def __init__(
            self, camera_selector: CameraSelector, camera_provider: CameraProvider,
            automator: Automator, detector: Detector,
            puncher: Puncher) -> None:
        self.log = logging.getLogger('field_friend.camera')
        self.camera_selector = camera_selector
        self.camera_provider = camera_provider
        self.camera: Camera = None
        self.automator = automator
        self.detector = detector
        self.capture_images = ui.timer(1, lambda: rosys.create_task(
            self.detector.upload(self.camera.latest_captured_image)), active=False)
        self.puncher = puncher
        self.image_view: ui.interactive_image = None
        self.calibration_dialog = calibration_dialog(camera_provider)
        with ui.card().tight().classes('col gap-4').style('width:600px') as self.card:
            ui.image('assets/field_friend.webp').classes('w-full')
            ui.label('no camera available').classes('text-center')
            self.camera_selector.CAMERA_SELECTED.register(self.use_camera)

    def on_mouse_move(self, e: MouseEventArguments):
        if e.type == 'mousemove':
            point2d = Point(x=e.image_x, y=e.image_y)
            point3d = self.camera.calibration.project_from_image(point2d)
            self.debug_position.set_text(f'{point2d} -> {point3d}')
        if e.type == 'mouseup':
            point2d = Point(x=e.image_x, y=e.image_y)
            point3d = self.camera.calibration.project_from_image(point2d)
            if point3d is not None:
                self.automator.start(self.puncher.drive_and_punch(point3d.x, point3d.y))
        if e.type == 'mouseout':
            self.debug_position.set_text('')

    def use_camera(self, camera: Camera) -> None:
        self.camera = camera
        self.card.clear()
        events = ['mousemove', 'mouseout', 'mouseup']
        with self.card:
            self.image_view = ui.interactive_image(
                self.camera_provider.get_latest_image_url(camera),
                cross=True,
                on_mouse=self.on_mouse_move,
                events=events
            ).classes('w-full')

            async def update():
                await self.image_view.set_source(self.camera_provider.get_latest_image_url(camera))

            ui.timer(1, update)
            with ui.row().classes('m-4 justify-end items-center'):
                ui.checkbox('Capture Images').bind_value_to(self.capture_images, 'active')\
                    .tooltip('Record new images for the Learning Loop')
                self.show_mapping = ui.checkbox('Show Mapping', on_change=self.show_mapping)\
                    .tooltip('Show the mapping between camera and world coordinates')
                ui.button('calibrate', on_click=self.calibrate) \
                    .props('icon=straighten outline').tooltip('Calibrate camera')
            with ui.row():
                self.debug_position = ui.label()

    async def calibrate(self) -> None:
        result = await self.calibration_dialog.edit(self.camera)
        if result:
            self.show_mapping.value = True

    def show_mapping(self, args: ValueChangeEventArguments) -> None:
        if not args.value:
            self.image_view.svg_content = ''
            return

        grid_points = np.array([p.tuple for p in self.create_grid_points()])
        grid_image_points = self.camera.calibration.project_array_to_image(world_points=grid_points)
        c = len(grid_image_points)

        def get_color(i: int) -> str:
            rgb = colorsys.hsv_to_rgb(i/c, 1, 1)
            return f"#{int(rgb[0]*255):02x}{int(rgb[1]*255):02x}{int(rgb[2]*255):02x}"
        self.image_view.set_svg_content(''.join(f'<circle cx="{g[0]}" cy="{g[1]}" r="2" fill="{get_color(i)}"/>' for i,
                                                g in enumerate(grid_image_points)))

    @staticmethod
    def create_grid_points() -> list[Point3d]:
        result = []
        x = 0
        while x <= 0.7:
            y = -0.2
            while y <= 0.2:
                result.append(Point3d(x=x, y=y, z=0))
                y += 0.03
            x += 0.03
        return result
