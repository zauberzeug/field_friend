from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import ui

# from ...hardware import FlashlightPWM
from .camera_card import CameraCard as camera_card
from .key_controls import KeyControls

if TYPE_CHECKING:
    from ...system import System


class Monitoring:
    def __init__(self, system: System, *, shrink_factor: int = 1) -> None:
        self.log = logging.getLogger('field_friend.monitoring')
        self.system = system
        self.usb_camera_provider = getattr(system, 'camera_provider', None)
        # TODO: in simulation there is no mjpeg camera provider
        self.mjpg_camera_provider = getattr(system, 'mjpeg_camera_provider', None)
        self.detector = getattr(system, 'detector', None)
        self.monitoring_detector = getattr(system, 'circle_sight_detector', None)
        self.monitoring_active = False
        self.plant_locator = getattr(system, 'plant_locator', None)
        self.capture = system.capture
        self.field_friend = system.field_friend
        self.automator = system.automator

        self.person_count = 0
        self.animal_count = 0
        self.shrink_factor = shrink_factor
        self.sights: dict[str, ui.interactive_image] = {}
        if system.config.circle_sight_positions is None:
            self.log.warning('No circle sight positions configured, camera views will not be available')
            self.camera_positions = None
        else:
            self.camera_positions = system.config.circle_sight_positions
        KeyControls(system)
        with ui.row().classes('w-full items-stretch items-center gap-2'):
            with ui.column().classes('w-1/6 items-center'):
                with ui.column() as self.front_view:
                    self._circle_sight_text('Front')
                    ui.interactive_image('assets/field_friend.webp').classes('w-full')
                with ui.column() as self.left_view:
                    self._circle_sight_text('Left')
                    ui.interactive_image('assets/field_friend.webp').classes('w-full')

            with ui.column().classes('w-[calc(65%)] items-center'):
                camera_card(self.system, shrink_factor=self.shrink_factor,
                            show_plants=True, show_detections=True, show_plants_to_handle=True)
                with ui.column().classes('w-full items-center'):
                    with ui.row().classes('w-full items-center'):
                        if self.monitoring_detector is not None:
                            ui.switch('Person detection') \
                                .bind_value(self, 'monitoring_active') \
                                .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                        if self.plant_locator is not None:
                            ui.switch('Plant detection') \
                                .bind_value(self.plant_locator, 'is_paused', forward=lambda x: not x, backward=lambda x: not x) \
                                .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)
                    if self.plant_locator is not None:
                        with ui.row().classes('w-full items-center'):
                            ui.button('Capture Outer', on_click=self.capture.circle_sight)
                            ui.button('Capture Front', on_click=lambda: self.capture.circle_sight(direction='front'))
                            ui.button('Capture Inner', on_click=self.capture.inner)

            with ui.column().classes('w-1/6 items-center'):
                with ui.column() as self.back_view:
                    self._circle_sight_text('Back')
                    ui.interactive_image('assets/field_friend.webp').classes('w-full')
                with ui.column() as self.right_view:
                    self._circle_sight_text('Right')
                    ui.interactive_image('assets/field_friend.webp').classes('w-full')
        ui.timer(0.1, self._update_monitor_content)

    def _circle_sight_text(self, position: str) -> None:
        ui.label(position).classes('w-full text-2xl text-bold text-center')

    async def _update_monitor_content(self):
        if self.mjpg_camera_provider is None:
            return
        for camera in self.mjpg_camera_provider.cameras.values():
            if not camera.is_connected:
                continue
            if camera.id in self.sights:
                self.sights[camera.id].set_source(camera.get_latest_image_url())
                continue
            if self.camera_positions is None:
                self.log.warning(f'Camera {camera.id} detected but no camera positions configured')
                continue
            if self.camera_positions.front in camera.id:
                self.front_view.clear()
                with self.front_view:
                    self._circle_sight_text('Front')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions.back in camera.id:
                self.back_view.clear()
                with self.back_view:
                    self._circle_sight_text('Back')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions.left in camera.id:
                self.left_view.clear()
                with self.left_view:
                    self._circle_sight_text('Left')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            elif self.camera_positions.right in camera.id:
                self.right_view.clear()
                with self.right_view:
                    self._circle_sight_text('Right')
                    self.sights[camera.id] = ui.interactive_image().classes('w-full')
            else:
                self.log.warning(f'Unknown camera position: {camera.id}')
                continue

        if self.monitoring_detector is None:
            return
        person_count = 0
        animal_count = 0
        for camera in self.mjpg_camera_provider.cameras.values():
            image = camera.latest_captured_image
            if not image:
                continue
            if self.monitoring_active:
                await self.monitoring_detector.detect(image, tags=[camera.id], autoupload=rosys.vision.Autoupload.DISABLED)
                if image.detections:
                    person_count += len([p for p in image.detections.points
                                        if p.category_name == 'person' and p.confidence > 0.4])
                    animal_count += len([p for p in image.detections.points
                                        if ('bird' in p.category_name or 'animal' in p.category_name) and p.confidence > 0.4])
                    self.sights[camera.id].set_content(self._to_svg(image.detections))
        self.person_count = person_count
        self.animal_count = animal_count

    def _to_svg(self, detections: rosys.vision.Detections) -> str:
        assert self.plant_locator is not None
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
