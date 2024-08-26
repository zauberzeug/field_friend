from __future__ import annotations

import io
import logging

import numpy as np
from nicegui import events, ui
from PIL import Image
from rosys.driving import Odometer
from rosys.geometry import Point3d
from rosys.vision import Calibration, Camera, CameraProvider, SimulatedCalibratableCamera

from ...vision import DOT_DISTANCE, Dot, Network


class calibration_dialog(ui.dialog):

    def __init__(self, camera_provider: CameraProvider, odometer: Odometer) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.calibration')
        self.odometer = odometer
        self.camera_provider = camera_provider
        self.network: Network | None = None
        self.calibration: Calibration | None = None
        self.mouse_x: float = 0
        self.mouse_y: float = 0
        self.moving_dot: Dot | None = None
        self.camera: Camera | None = None
        with self, ui.card().tight().style('max-width: 1000px'):
            ui.keyboard(self.handle_key)
            self.calibration_image = ui.interactive_image(on_mouse=self.handle_mouse,
                                                          events=['mousemove', 'mousedown', 'mouseup'],
                                                          cross=True)
            with ui.dialog() as help_dialog, ui.card():
                ui.markdown('''
                    1. Drag points to lock them to their corresponding contours.
                    
                        You can lock a point without a contour by pressing "Enter". 
                        
                        This will turn the point green.
                    
                        You can unlock a point by pressing "Delete" or "Backspace". 
                            
                        This will turn the point orange.
                    
                    2. Use arrow keys to shift the entire grid to the correct origin.
                            
                        (0,0,0) point should be on the marked center of the calibration plate

                    3. Click "Calibrate" to run the image calibration. 
                            
                        Red dots will appear at the projected positions of the grid points.
                            
                    4. Click "Apply Calibration" to save the calibration to the camera.
                ''')

            with ui.row().classes('p-4 w-full items-center'):
                self.focal_length_input = ui.number('Focal length', value=1830, step=10).classes('w-24')
                ui.button('Calibrate', icon='straighten', on_click=lambda: self.set_calibration(
                    self.network.calibrate(self.focal_length_input.value, self.odometer.prediction_frame))).props('outline')
                ui.button('Help', icon='sym_o_help', on_click=help_dialog.open).props('outline')
                ui.space()
                ui.button('Cancel', on_click=self.close).props('color=warning outline')
                ui.button('Apply Calibration', on_click=self.apply_calibration).bind_enabled(self, 'calibration')

    async def edit(self, camera: Camera) -> bool:
        self.log.info(f'editing camera calibration for: {camera.id}')
        self.camera = camera
        if not isinstance(camera, SimulatedCalibratableCamera):
            image = camera.latest_captured_image
            if image is None:
                self.log.warning('No image available')
                return False
            if camera.calibration:
                self.calibration = camera.calibration
            self.calibration_image.source = camera.get_latest_image_url()
            img = Image.open(io.BytesIO(image.data))
        else:
            img = Image.open('assets/image.jpg')
            image = ui.image(img)
            self.calibration_image.source = image.source
        if camera.focal_length is None:
            camera.focal_length = 1830
        self.focal_length_input.bind_value(camera, 'focal_length')
        self.network = Network.from_img(np.array(img))
        self.render()
        self.open()
        return (await self) or False

    def render(self) -> None:
        if not self.network:
            return
        content = ''

        for contour in self.network.contours:
            points = ','.join(f'{p[0][0]} {p[0][1]}' for p in contour.points)
            content += f'<polygon points="{points}" stroke="#00ff00" fill="none" stroke-width="2" />'

        dots = self.network.dots
        for i, j, k in dots:
            for di, dj, dk in [
                (1, 0, 0),
                (0, 1, 0),
                (0, 0, 1),
                (0.5, 0.5, 0.5),
                (-0.5, 0.5, 0.5),
                (0.5, -0.5, 0.5),
                (-0.5, -0.5, 0.5),
            ]:
                if (i + di, j + dj, k + dk) in dots:
                    dot0 = dots[(i, j, k)]
                    dot1 = dots[(i + di, j + dj, k + dk)]
                    content += f'<line x1="{dot0.x}" y1="{dot0.y}" x2="{dot1.x}" y2="{dot1.y}" stroke="white" />'

        for (i, j, k), dot in dots.items():
            color = '#00ff00' if dot.is_refined else '#ff8800'
            radius = '8' if (i, j, k) == (0, 0, 0) else '5'
            content += f'<circle cx="{dot.x}" cy="{dot.y}" r="{radius}" fill="{color}" />'

            if not dot.is_refined:
                continue
            if (i, j, k) == (0, 0, 0):  # This is the center point
                # Use a different, more visible color for the center point text and increase font size
                text_color = '#ff0000'  # Bright red for visibility
                font_size = '18'  # Larger font size for better readability
                content += f'<text x="{dot.x+10}" y="{dot.y+10}" dominant-baseline="hanging" fill="{text_color}" font-size="{font_size}">({i}, {j}, {k})</text>'
            else:
                # Existing logic for other points
                color = '#000' if dot.is_refined else '#666'  # Keep existing color logic for other points
                content += f'<text x="{dot.x+10}" y="{dot.y+10}" dominant-baseline="hanging" fill="{color}">({i}, {j}, {k})</text>'

            if not self.calibration:
                continue

            world_point = Point3d(x=i * DOT_DISTANCE, y=j * DOT_DISTANCE, z=k * DOT_DISTANCE)
            self.log.info(
                f'world point: {world_point} and image point: {self.calibration.project_to_image(world_point)}')
            image_point = self.calibration.project_to_image(world_point)
            if image_point is not None:
                content += f'<circle cx="{image_point.x}" cy="{image_point.y}" r="5" fill="red" />'

        self.calibration_image.content = content

    def handle_mouse(self, e: events.MouseEventArguments) -> None:
        self.mouse_x = e.image_x
        self.mouse_y = e.image_y
        if e.type == 'mousedown':
            self.moving_dot = self.find_dot(e.image_x, e.image_y)
        if e.type in {'mousedown', 'mousemove'}:
            if self.moving_dot:
                self.moving_dot.x = e.image_x
                self.moving_dot.y = e.image_y
                self.render()
        if e.type == 'mouseup':
            if self.moving_dot is not None:
                self.network.try_refine(self.moving_dot)
                self.network.auto_grow()
                self.render()
            self.moving_dot = None

    def handle_key(self, e: events.KeyEventArguments) -> None:
        if not e.action.keydown:
            return
        if e.key.delete or e.key.backspace:
            dot = self.find_dot(self.mouse_x, self.mouse_y)
            if dot:
                dot.is_refined = False
                self.render()
        if e.key.enter:
            dot = self.find_dot(self.mouse_x, self.mouse_y)
            if dot:
                dot.is_refined = True
                self.network.auto_grow()
                self.render()
        if (e.key.arrow_up or e.key.arrow_down or e.key.arrow_left or e.key.arrow_right):
            self.network.shift(di=1 if e.key.arrow_up else -1 if e.key.arrow_down else 0,
                               dj=1 if e.key.arrow_left else -1 if e.key.arrow_right else 0)
            self.render()

    def find_dot(self, x: float, y: float) -> Dot | None:
        distances = {key: np.sqrt((dot.x - x)**2 + (dot.y - y)**2) for key, dot in self.network.dots.items()}
        if not distances or min(distances.values()) > 100:
            return None
        return self.network.dots[min(distances, key=distances.get)]  # type: ignore

    def set_calibration(self, calibration: Calibration | None) -> None:
        self.calibration = calibration
        self.render()

    def apply_calibration(self) -> None:
        if not self.camera:
            return
        try:
            if not self.calibration:
                raise ValueError('No calibration created')
            self.camera.calibration = self.calibration
        except Exception as e:
            self.camera.calibration = None
            ui.notify(str(e))
        else:
            ui.notify('Calibration applied')
            self.camera_provider.request_backup()
            self.submit(True)
