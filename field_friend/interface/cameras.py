import colorsys
import logging
from typing import Optional

import numpy as np
import rosys
from nicegui import app, ui
from nicegui.elements.card import Card
from nicegui.events import MouseEventArguments, ValueChangeEventArguments
from rosys import background_tasks
from rosys.automation import Automator
from rosys.geometry import Point, Point3d
from rosys.vision import Camera, CameraProvider, Detector

from ..automations import Puncher
from ..vision import CameraSelector
from .calibration_dialog import calibration_dialog


class CameraCard(Card):

    def __init__(self, camera_type: str, camera_provider: CameraProvider, camera_selector: CameraSelector,
                 automator: Automator, detector: Detector, puncher: Optional[Puncher] = None) -> None:
        super().__init__()
        self.log = logging.getLogger(f'rosys.camera_card')
        self.camera_type = camera_type
        self.camera = None
        self.camera_provider = camera_provider
        self.camera_selector = camera_selector
        self.automator = automator
        self.detector = detector
        self.capture_images = ui.timer(1, lambda: background_tasks.create(
            self.detector.upload(self.camera.latest_captured_image)), active=False)
        self.puncher = puncher
        self.image_view: ui.interactive_image = None
        self.calibration_dialog = calibration_dialog(camera_provider)
        with self.tight().classes('col gap-4').style('width:640px'):
            app.add_static_files('/assets', 'assets')
            ui.image('assets/field_friend.webp').classes('w-full')
            ui.label(f'no {camera_type} available').classes('text-center')
        self.camera_selector.CAMERA_SELECTED.register(self.use_camera)

    def use_camera(self, camera_data: tuple[str, Camera]) -> None:
        camera_type, camera = camera_data
        if camera_type != self.camera_type:
            self.log.info(f'ignoring camera {camera_type} (expected {self.camera_type})')
            return
        self.camera = camera
        self.clear()
        events = ['mousemove', 'mouseout', 'mouseup']
        with self:
            with ui.row().classes('w-full items-center').style('gap:0.5em;margin-left:1em;margin-right:1em'):
                ui.label(f'{camera_type}:').classes('text-xl')
            self.image_view = ui.interactive_image(
                self.camera_provider.get_latest_image_url(camera),
                cross=True,
                on_mouse=self.on_mouse_move,
                events=events
            ).classes('w-full')

            def update():
                self.image_view.set_source(self.camera_provider.get_latest_image_url(camera))

            ui.timer(0.5, update)
            with ui.row().classes('m-4 justify-end items-center'):
                ui.checkbox('Capture Images').bind_value_to(self.capture_images, 'active') \
                    .tooltip('Record new images for the Learning Loop')
                self.show_mapping = ui.checkbox('Show Mapping', on_change=self.show_mapping) \
                    .tooltip('Show the mapping between camera and world coordinates')
                ui.button('calibrate', on_click=self.calibrate) \
                    .props('icon=straighten outline').tooltip('Calibrate camera')
            with ui.row():
                self.debug_position = ui.label()

    def on_mouse_move(self, e: MouseEventArguments):
        if e.type == 'mousemove':
            point2d = Point(x=e.image_x, y=e.image_y)
            point3d = self.camera.calibration.project_from_image(point2d)
            self.debug_position.set_text(f'{point2d} -> {point3d}')
        if e.type == 'mouseup':
            point2d = Point(x=e.image_x, y=e.image_y)
            point3d = self.camera.calibration.project_from_image(point2d)
            if point3d is not None and self.puncher is not None:
                self.automator.start(self.puncher.drive_and_punch(point3d.x, point3d.y))
        if e.type == 'mouseout':
            self.debug_position.set_text('')

    async def calibrate(self) -> None:
        result = await self.calibration_dialog.edit(self.camera)
        if result:
            self.show_mapping.value = True

    def show_mapping(self, args: ValueChangeEventArguments) -> None:
        if not args.value:
            self.image_view.content = ''
            return

        grid_points = np.array([p.tuple for p in self.create_grid_points()])
        grid_image_points = self.camera.calibration.project_array_to_image(world_points=grid_points)
        c = len(grid_image_points)

        def get_color(i: int) -> str:
            rgb = colorsys.hsv_to_rgb(i/c, 1, 1)
            return f"#{int(rgb[0]*255):02x}{int(rgb[1]*255):02x}{int(rgb[2]*255):02x}"
        self.image_view.set_content(''.join(f'<circle cx="{g[0]}" cy="{g[1]}" r="2" fill="{get_color(i)}"/>' for i,
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


class cameras:

    def __init__(
            self, camera_selector: CameraSelector, camera_provider: CameraProvider,
            automator: Automator, detector: Detector,
            puncher: Optional[Puncher] = None) -> None:
        self.log = logging.getLogger('field_friend.cameras')
        self.camera_selector = camera_selector
        self.camera_provider = camera_provider
        self.automator = automator
        self.detector = detector
        self.puncher = puncher
        self.cards: dict[str, CameraCard] = {}

        # with ui.row() as self.camera_grid:
        #     for camera_type in self.camera_selector.camera_ids.keys():
        #         CameraCard(camera_type, self.camera_provider, self.camera_selector,
        #                    self.automator, self.detector, self.puncher)
        self.camera_grid = ui.row()
        ui.timer(1, self.update_cameras)

    def update_cameras(self) -> None:
        with self.camera_grid:
            if set(self.camera_selector.camera_ids.keys()) != set(self.cards.keys()):
                self.camera_grid.clear()
                for camera_type in self.camera_selector.camera_ids.keys():
                    self.cards[camera_type] = CameraCard(camera_type, self.camera_provider, self.camera_selector,
                                                         self.automator, self.detector, self.puncher)
