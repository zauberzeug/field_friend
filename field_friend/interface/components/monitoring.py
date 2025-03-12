from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import events, ui

import config.config_selection as config_selector

from ...hardware import FlashlightPWM
from .key_controls import KeyControls

if TYPE_CHECKING:
    from ...system import System


class CameraPosition:
    RIGHT = '-1'
    BACK = '-2'
    FRONT = '-3'
    LEFT = '-4'


class Monitoring:

    def __init__(self, system: System, *,
                 shrink_factor: int = 1) -> None:
        self.log = logging.getLogger('field_friend.monitoring')
        # self.usb_camera_provider = system.camera_provider
        # TODO: in simulation there is no mjpeg camera provider
        self.mjpg_camera_provider = system.mjpeg_camera_provider
        # self.detector = system.detector
        # self.monitoring_detector = system.monitoring_detector
        self.monitoring_active = False
        self.plant_locator = system.plant_locator
        self.field_friend = system.field_friend
        self.automator = system.automator
        self.system = system
        self.person_count = 0
        self.animal_count = 0
        self.shrink_factor = shrink_factor
        self.sights: dict[str, ui.interactive_image] = {}
        self.camera_positions = self.load_camera_positions()
        KeyControls(system)
        ui.keyboard(self.handle_key)
        with ui.row().classes('w-full items-stretch gap-0'):
            with ui.column().classes('w-1/4 items-center') as self.left_view:
                ui.label('Left').classes('text-2xl text-bold')
                ui.image('assets/field_friend.webp').classes('w-full')
            with ui.column().classes('w-1/4 items-center') as self.front_view:
                ui.label('Front').classes('text-2xl text-bold')
                ui.image('assets/field_friend.webp').classes('w-full')
            with ui.column().classes('w-1/4 items-center') as self.back_view:
                ui.label('Back').classes('text-2xl text-bold')
                ui.image('assets/field_friend.webp').classes('w-full')
            with ui.column().classes('w-1/4 items-center') as self.right_view:
                ui.label('Right').classes('text-2xl text-bold')
                ui.image('assets/field_friend.webp').classes('w-full')
        with ui.row().classes('w-full items-stretch gap-0'):
            with ui.card().style('background-color: #6E93D6; color: white;').classes('w-full'):
                with ui.row().classes('w-full items-stretch'):
                    ui.label('Person count:').classes('text-2xl text-bold').bind_text_from(self,
                                                                                           'person_count', backward=lambda x: f'Person count: {x}')
                    ui.label('Animal count:').classes('text-2xl text-bold').bind_text_from(self,
                                                                                           'animal_count', backward=lambda x: f'Animal count: {x}')
                    ui.space()
                    # ui.switch('Person detection') \
                    #     .bind_value(self, 'monitoring_active') \
                    #     .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                    # ui.switch('Plant detection') \
                    #     .bind_value(self.plant_locator, 'is_paused', forward=lambda x: not x, backward=lambda x: not x) \
                    #     .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)

        with ui.row().classes('w-full items-stretch gap-0'):
            column_classes = 'w-1/3 items-center mt-[50px]'
            text_style = 'font-size: 15em; line-height: 80%;'
            with ui.column().classes(column_classes):
                self.crops_count_label = ui.label().style(text_style)
                self.crops_label = ui.label('Crops').classes('text-2xl text-bold')
            with ui.column().classes('w-1/3'):
                with ui.interactive_image() as self.bottom_view:
                    if self.field_friend.flashlight and isinstance(self.field_friend.flashlight, FlashlightPWM):
                        self.flashlight_toggled = False

                        async def toggle_flashlight():
                            if not self.field_friend.flashlight:
                                rosys.notify('No flashlight found')
                                return
                            self.flashlight_toggled = not self.flashlight_toggled
                            flashlight_button.props(
                                f'flat color={"primary" if not self.flashlight_toggled else "grey"} icon={"flashlight_on" if not self.flashlight_toggled else "flashlight_off"}')
                            if self.flashlight_toggled:
                                await self.field_friend.flashlight.turn_on()
                                rosys.notify('Flashlight turned on')
                            else:
                                await self.field_friend.flashlight.turn_off()
                                rosys.notify('Flashlight turned off')
                        flashlight_button = ui.button(icon='flashlight_on', on_click=toggle_flashlight).props('flat color=primary').style(
                            'position: absolute; left: 1px; top: 1px; z-index: 500;').bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
            with ui.column().classes(column_classes):
                self.weeds_count_label = ui.label().style(text_style)
                self.weeds_label = ui.label('Weeds').classes('text-2xl text-bold')

        ui.timer(0.1, self.update_monitor_content)
        # ui.timer(0.1, self.update_bottom_view)

    async def update_monitor_content(self):
        for camera in self.mjpg_camera_provider.cameras.values():
            if camera.id in self.sights or not camera.is_connected:
                continue
            if self.camera_positions['front'] in camera.id:
                self.front_view.clear()
                with self.front_view:
                    ui.label('Front').classes('text-2xl text-bold')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions['back'] in camera.id:
                self.back_view.clear()
                with self.back_view:
                    ui.label('Back').classes('text-2xl text-bold')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions['left'] in camera.id:
                self.left_view.clear()
                with self.left_view:
                    ui.label('Left').classes('text-2xl text-bold')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions['right'] in camera.id:
                self.right_view.clear()
                with self.right_view:
                    ui.label('Right').classes('text-2xl text-bold')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            else:
                self.log.warning(f'Unknown camera position: {camera.id}')
                continue

        person_count = 0
        animal_count = 0
        for camera in self.mjpg_camera_provider.cameras.values():
            image = camera.latest_captured_image
            if not image:
                continue
            self.sights[camera.id].set_source(camera.get_latest_image_url())
            if self.monitoring_active:
                pass
                # await self.monitoring_detector.detect(image, tags=[camera.id], autoupload=rosys.vision.Autoupload.DISABLED)
                # if image.detections:
                #     person_count += len([p for p in image.detections.points
                #                         if p.category_name == 'person' and p.confidence > 0.4])
                #     animal_count += len([p for p in image.detections.points
                #                         if ('bird' in p.category_name or 'animal' in p.category_name) and p.confidence > 0.4])
                #     self.sights[camera.id].set_content(self.to_svg(image.detections))
        self.person_count = person_count
        self.animal_count = animal_count

    async def update_bottom_view(self):
        cameras = list(self.usb_camera_provider.cameras.values())
        camera = next((camera for camera in cameras if camera.is_connected), None)
        if not camera:
            self.bottom_view.set_source('assets/field_friend.webp')
            return
        image = camera.latest_captured_image if self.plant_locator and self.plant_locator.is_paused \
            else camera.latest_detected_image
        if not image:
            return
        source = camera.get_latest_image_url()
        self.bottom_view.set_source(f'{source}?shrink={self.shrink_factor}')
        if image.detections:
            self.bottom_view.set_content(self.to_svg(image.detections))
            crops = len([p for p in image.detections.points if p.category_name in self.plant_locator.crop_category_names and p.confidence >
                        self.plant_locator.minimum_crop_confidence])
            weeds = len([p for p in image.detections.points if p.category_name in self.plant_locator.weed_category_names and p.confidence >
                        self.plant_locator.minimum_weed_confidence])
            self.crops_count_label.set_text(str(crops))
            self.crops_label.set_text('Crop' if crops == 1 else 'Crops')
            self.weeds_count_label.set_text(str(weeds))
            self.weeds_label.set_text('Weed' if weeds == 1 else 'Weeds')
        else:
            self.weeds_count_label.set_text('0')
            self.crops_count_label.set_text('0')
            self.bottom_view.set_content('')

    def to_svg(self, detections: rosys.vision.Detections) -> str:
        svg = ''
        cross_size = 20
        for point in detections.points:
            if point.category_name == 'person' and point.confidence > 0.3:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="red" />'
                svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="red">Person</text>'
            elif 'bird' in point.category_name and point.confidence > 0.3:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="orange" />'
                svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="blue">Bird</text>'
            elif 'animal' in point.category_name and point.confidence > 0.3:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="orange" />'
                svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="green">Animal</text>'
            elif point.category_name in self.plant_locator.crop_category_names and point.confidence > self.plant_locator.minimum_crop_confidence:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="18" stroke-width="8" stroke="green" fill="none" />'
                svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="green">Crop</text>'
            elif point.category_name in self.plant_locator.weed_category_names and point.confidence > self.plant_locator.minimum_weed_confidence:
                svg += f'''
                        <line x1="{point.x / self.shrink_factor - cross_size}" y1="{point.y / self.shrink_factor}" x2="{point.x / self.shrink_factor + cross_size}" y2="{point.y / self.shrink_factor}" stroke="red" stroke-width="8"
                            transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                        <line x1="{point.x / self.shrink_factor}" y1="{point.y / self.shrink_factor - cross_size}" x2="{point.x / self.shrink_factor}" y2="{point.y / self.shrink_factor + cross_size}" stroke="red" stroke-width="8"
                            transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                        <text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="red">Weed</text>
                '''
            else:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="yellow" />'
                svg += f'<text x="{point.x / self.shrink_factor-30}" y="{point.y / self.shrink_factor+30}" font-size="20" fill="yellow">{point.category_name}</text>'
        return svg

    def handle_key(self, e: events.KeyEventArguments) -> None:
        if e.modifiers.shift and e.key.is_cursorkey:
            if e.key.arrow_up:
                # Show front view and adjust its width
                self.front_view.classes(
                    remove='hidden', add='w-1/2').style(add='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
                # Hide back view and reset its width if needed
                self.back_view.classes(
                    add='hidden', remove='w-1/2').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            elif e.key.arrow_down:
                # Show back view and adjust its width
                self.back_view.classes(
                    remove='hidden', add='w-1/2').style(add='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
                # Hide front view and reset its width if needed
                self.front_view.classes(
                    add='hidden', remove='w-1/2').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            if e.key.arrow_left:
                # Show left view and adjust its width
                self.left_view.classes(
                    remove='hidden', add='w-1/2').style(add='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
                # Hide right view and reset its width if needed
                self.right_view.classes(
                    add='hidden', remove='w-1/2').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            elif e.key.arrow_right:
                # Show right view and adjust its width
                self.right_view.classes(
                    remove='hidden', add='w-1/2').style(add='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
                # Hide left view and reset its width if needed
                self.left_view.classes(
                    add='hidden', remove='w-1/2').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
        if e.action.keyup:
            self.front_view.classes(remove='hidden w-1/2',
                                    add='w-1/4').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            self.back_view.classes(remove='hidden w-1/2',
                                   add='w-1/4').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            self.left_view.classes(remove='hidden w-1/2',
                                   add='w-1/4').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')
            self.right_view.classes(remove='hidden w-1/2',
                                    add='w-1/4').style(remove='border: 5px solid #6E93D6; border-radius: 5px; background-color: #6E93D6; color: white')

    def load_camera_positions(self) -> dict:
        config = config_selector.import_config(module='camera')
        config_positions = config.get('circle_sight_positions', {})
        return {
            'left': config_positions.get('left', CameraPosition.LEFT),
            'right': config_positions.get('right', CameraPosition.RIGHT),
            'front': config_positions.get('front', CameraPosition.FRONT),
            'back': config_positions.get('back', CameraPosition.BACK),
        }
