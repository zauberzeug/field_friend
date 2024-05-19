import colorsys
import logging
from typing import Optional

import numpy as np
import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments, ValueChangeEventArguments
from rosys.geometry import Point

from ...automations import PlantLocator, Puncher
from ...hardware import FieldFriend, FlashlightPWM, FlashlightPWMV2, Tornado, ZAxis
from .calibration_dialog import calibration_dialog


class camera_card:

    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider,
                 automator: rosys.automation.Automator,
                 detector: rosys.vision.Detector,
                 plant_locator: PlantLocator,
                 field_friend: FieldFriend,
                 puncher: Optional[Puncher] = None,
                 *,
                 shrink_factor: int = 1) -> None:
        self.log = logging.getLogger('field_friend.camera_card')
        self.camera: Optional[rosys.vision.CalibratableCamera] = None
        self.camera_provider = camera_provider
        self.automator = automator
        self.detector = detector
        self.plant_locator = plant_locator
        self.punching_enabled = False
        self.puncher = puncher
        self.field_friend = field_friend
        self.shrink_factor = shrink_factor
        self.image_view: Optional[ui.interactive_image] = None
        self.calibration_dialog = calibration_dialog(camera_provider)
        self.camera_card = ui.card()
        with self.camera_card.tight().classes('w-full'):
            ui.label('no camera available').classes('text-center')
            ui.image('assets/field_friend.webp').classes('w-full')
        ui.timer(0.2, self.update_content)

    def use_camera(self, cam: rosys.vision.CalibratableCamera) -> None:
        self.camera = cam
        self.camera_card.clear()
        with self.camera_card.style('position: relative;'):
            if self.field_friend.flashlight and (isinstance(self.field_friend.flashlight, FlashlightPWM) or isinstance(self.field_friend.flashlight, FlashlightPWMV2)):
                self.flashlight_toggled = False

                async def toggle_flashlight():
                    self.flashlight_toggled = not self.flashlight_toggled
                    flashlight_button.props(
                        f'flat color={"grey" if not self.flashlight_toggled else "primary"} icon={"flashlight_off" if not self.flashlight_toggled else "flashlight_on"}')
                    if self.flashlight_toggled:
                        await self.field_friend.flashlight.turn_on()
                        rosys.notify('Flashlight turned on')
                    else:
                        await self.field_friend.flashlight.turn_off()
                        rosys.notify('Flashlight turned off')
                flashlight_button = ui.button(icon='flashlight_off', on_click=toggle_flashlight).props('flat color=grey').style(
                    'position: absolute; left: 1px; top: 1px; z-index: 500;').bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)

            with ui.button(icon='menu').props('flat color=primary').style('position: absolute; right: 1px; top: 1px; z-index: 500;'):
                with ui.menu() as menu:
                    with ui.menu_item():
                        with ui.row():
                            ui.checkbox('Punching').bind_value(self, 'punching_enabled').tooltip(
                                'Enable punching mode').bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                            if isinstance(self.field_friend.z_axis, ZAxis):
                                self.depth = ui.number('depth', value=0.02, format='%.2f',
                                                       step=0.01, min=self.field_friend.z_axis.max_position, max=-self.field_friend.z_axis.min_position).classes('w-16').bind_visibility_from(self, 'punching_enabled')
                            elif isinstance(self.field_friend.z_axis, Tornado):
                                self.angle = ui.number('angle', value=180, format='%.0f', step=1, min=0, max=180).classes(
                                    'w-16').bind_visibility_from(self, 'punching_enabled')
                    with ui.menu_item():
                        ui.checkbox('Detecting Plants').bind_value(self.plant_locator, 'is_paused',
                                                                   backward=lambda x: not x, forward=lambda x: not x) \
                            .tooltip('Pause plant locator').bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                    with ui.menu_item():
                        self.show_mapping_checkbox = ui.checkbox('Mapping', on_change=self.show_mapping) \
                            .tooltip('Show the mapping between camera and world coordinates')
                    with ui.menu_item():
                        ui.button('calibrate', on_click=self.calibrate) \
                            .props('icon=straighten outline').tooltip('Calibrate camera')
                    # TODO: ist das hier ein todo?: Add a button to save the last captured image
                    # ui.button('Save Image', on_click=self.save_last_image).classes('m-2')

            events = ['mousemove', 'mouseout', 'mouseup']
            self.image_view = ui.interactive_image(
                '',
                cross=True,
                on_mouse=self.on_mouse_move,
                events=events
            ).classes('w-full')
            with ui.row():
                self.debug_position = ui.label()

    def update_content(self) -> None:
        cameras = list(self.camera_provider.cameras.values())
        active_camera = next((camera for camera in cameras if camera.is_connected), None)
        if not active_camera:
            if self.camera:
                self.camera = None
                self.camera_card.clear()
                with self.camera_card:
                    ui.label('no camera available').classes('text-center')
                    ui.image('assets/field_friend.webp').classes('w-full')
            return
        if self.camera is None or self.camera != active_camera:
            self.use_camera(active_camera)
        if self.shrink_factor > 1:
            url = f'{self.camera.get_latest_image_url()}?shrink={self.shrink_factor}'
        else:
            url = self.camera.get_latest_image_url()
        self.image_view.set_source(url)
        image = self.camera.latest_detected_image
        if image and image.detections:
            self.image_view.set_content(self.to_svg(image.detections))

    def on_mouse_move(self, e: MouseEventArguments):
        if self.camera is None:
            return
        if e.type == 'mousemove':
            point2d = Point(x=e.image_x, y=e.image_y)
            if self.camera.calibration is None:
                self.debug_position.set_text(f'{point2d} no calibration')
                return
            point3d = self.camera.calibration.project_from_image(point2d)
            self.debug_position.set_text(f'{point2d} -> {point3d}')
        if e.type == 'mouseup':
            point2d = Point(x=e.image_x, y=e.image_y)
            if self.camera.calibration is None:
                self.debug_position.set_text(f'last punch: {point2d}')
                return
            point3d = self.camera.calibration.project_from_image(point2d)
            if point3d is not None:
                self.debug_position.set_text(f'last punch: {point2d} -> {point3d}')
                if self.puncher is not None and self.punching_enabled:
                    self.log.info(f'punching {point3d}')
                    if isinstance(self.field_friend.z_axis, ZAxis):
                        self.automator.start(self.puncher.drive_and_punch(point3d.x, point3d.y, self.depth.value))
                    elif isinstance(self.field_friend.z_axis, Tornado):
                        self.automator.start(self.puncher.drive_and_punch(
                            point3d.x, point3d.y, angle=self.angle.value))
        if e.type == 'mouseout':
            self.debug_position.set_text('')

    async def calibrate(self) -> None:
        result = await self.calibration_dialog.edit(self.camera)
        if result:
            self.show_mapping_checkbox.value = True
            self.camera_provider.request_backup()

    def show_mapping(self, event: ValueChangeEventArguments) -> None:
        if self.camera is None or self.image_view is None:
            return
        if not event.value:
            self.image_view.content = ''
            return
        if self.camera.calibration is None:
            rosys.notify('No calibration, calibrate camera first', 'negative')
            return
        world_points = np.array([[x, y, 0] for x in np.linspace(-0.3, 0.3, 15) for y in np.linspace(-0.25, 0.25, 20)])
        image_points = self.camera.calibration.project_to_image(world_points)
        colors_rgb = [colorsys.hsv_to_rgb(f, 1, 1) for f in np.linspace(0, 1, len(world_points))]
        colors_hex = [f'#{int(rgb[0] * 255):02x}{int(rgb[1] * 255):02x}{int(rgb[2] * 255):02x}' for rgb in colors_rgb]
        self.image_view.content = ''.join(f'<circle cx="{p[0]}" cy="{p[1]}" r="2" fill="{color}"/>'
                                          for p, color in zip(image_points, colors_hex))

    def to_svg(self, detections: rosys.vision.Detections) -> str:
        svg = ''
        cross_size = 20
        for point in detections.points:
            if point.category_name in self.plant_locator.crop_category_names:
                if point.confidence > self.plant_locator.minimum_crop_confidence:
                    svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="18" stroke-width="8" stroke="green" fill="none" />'
                    svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="green">Crop</text>'
                else:
                    svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="18" stroke-width="8" stroke="orange" fill="none" />'
                    svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="orange">{point.category_name}</text>'
            elif point.category_name in self.plant_locator.weed_category_names:
                if point.confidence > self.plant_locator.minimum_weed_confidence:
                    svg += f'''
                            <line x1="{point.x / self.shrink_factor - cross_size}" y1="{point.y / self.shrink_factor}" x2="{point.x / self.shrink_factor + cross_size}" y2="{point.y / self.shrink_factor}" stroke="red" stroke-width="8" 
                                transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                            <line x1="{point.x / self.shrink_factor}" y1="{point.y / self.shrink_factor - cross_size}" x2="{point.x / self.shrink_factor}" y2="{point.y / self.shrink_factor + cross_size}" stroke="red" stroke-width="8" 
                                transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                            <text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="red">Weed</text>
                    '''
                else:
                    svg += f'''
                            <line x1="{point.x / self.shrink_factor - cross_size}" y1="{point.y / self.shrink_factor}" x2="{point.x / self.shrink_factor + cross_size}" y2="{point.y / self.shrink_factor}" stroke="yellow" stroke-width="8" 
                                transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                            <line x1="{point.x / self.shrink_factor}" y1="{point.y / self.shrink_factor - cross_size}" x2="{point.x / self.shrink_factor}" y2="{point.y / self.shrink_factor + cross_size}" stroke="yellow" stroke-width="8" 
                                transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                            <text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="yellow">Weed</text>
                    '''
        return svg

    # async def save_last_image(self) -> None:
    #     """Saves the last captured image to the .rosys folder."""
    #     if self.camera and self.camera.latest_captured_image:
    #         image = self.camera.latest_captured_image
    #         self.log.info(f'Image captured at {image.size}')
    #         img = Image.open(io.BytesIO(image.data))
    #         # Resolves to the user's home directory  # modify the file name as needed
    #         backup_path = Path('~/.rosys').expanduser()
    #         save_path = backup_path / 'left_image.jpg'  # Modify the file name as needed
    #         try:
    #             # Use the save method of the image
    #             img.save(save_path)
    #             self.log.info(f'Image saved to {save_path}')
    #         except Exception as e:
    #             self.log.error(f'Error saving image: {e}')

    #     else:
    #         self.log.warning('No image available to save.')
