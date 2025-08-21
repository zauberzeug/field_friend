from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import ui

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

        self.shrink_factor = shrink_factor
        self.sights: dict[str, ui.interactive_image] = {}
        if system.config.circle_sight_positions is None:
            self.log.warning('No circle sight positions configured, camera views will not be available')
            self.circle_sight_positions = None
        else:
            self.circle_sight_positions = system.config.circle_sight_positions
        KeyControls(system)
        with ui.row().classes('w-full h-1/2 items-stretch items-center gap-2'):
            with ui.column().classes('w-1/6 items-center'):
                self._circle_sight_text('Front')
                self.sights['front'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                self._circle_sight_text('Left')
                self.sights['left'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                if self.monitoring_detector is not None:
                    ui.switch('Person detection') \
                        .bind_value(self, 'monitoring_active') \
                        .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)

            with ui.column().classes('w-[calc(65%)] items-center'):
                with ui.row().classes('items-center'):
                    camera_card(self.system, shrink_factor=self.shrink_factor,
                                show_plants=True, show_detections=True, show_plants_to_handle=True)
                with ui.row(align_items='center'):
                    if self.monitoring_detector is not None:
                        ui.button('Capture Outer', on_click=self.capture.circle_sight)
                        ui.button('Capture Front', on_click=lambda: self.capture.circle_sight(direction='front'))
                    if self.plant_locator is not None:
                        ui.button('Capture Inner', on_click=self.capture.inner)

            with ui.column().classes('w-1/6 items-center'):
                self._circle_sight_text('Back')
                self.sights['back'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                self._circle_sight_text('Right')
                self.sights['right'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
        ui.timer(0.1, self._update_monitor_content)

    def _circle_sight_text(self, position: str) -> None:
        ui.label(position).classes('w-full text-2xl text-bold text-center')

    async def _update_monitor_content(self):
        def camera_id_to_position(camera_id: str) -> str | None:
            assert self.circle_sight_positions is not None
            if camera_id.endswith(self.circle_sight_positions.front):
                return 'front'
            if camera_id.endswith(self.circle_sight_positions.back):
                return 'back'
            if camera_id.endswith(self.circle_sight_positions.left):
                return 'left'
            if camera_id.endswith(self.circle_sight_positions.right):
                return 'right'
            return None

        if self.mjpg_camera_provider is None or self.circle_sight_positions is None:
            return
        for camera in self.mjpg_camera_provider.cameras.values():
            if not camera.is_connected:
                continue
            if position := camera_id_to_position(camera.id):
                self.sights[position].set_source(camera.get_latest_image_url())
            else:
                self.log.warning(f'Unknown camera position: {camera.id}')
                continue
        if self.monitoring_detector is None:
            return
        for camera in self.mjpg_camera_provider.cameras.values():
            image = camera.latest_captured_image
            if not image:
                continue
            if self.monitoring_active and self.monitoring_detector.is_connected:
                position = camera_id_to_position(camera.id)
                if position is None:
                    continue
                await self.monitoring_detector.detect(image, tags=[position], autoupload=rosys.vision.Autoupload.FILTERED)
                if image.detections:
                    self.sights[position].set_content(self._to_svg(image.detections))

    def _to_svg(self, detections: rosys.vision.Detections) -> str:
        def point_to_svg(point: rosys.vision.PointDetection, *, color: str, radius: float = 8) -> str:
            return f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="{radius/self.shrink_factor}" fill="{color}" />'

        def box_to_svg(box: rosys.vision.BoxDetection, *, color: str) -> str:
            return f'<rect x="{box.x / self.shrink_factor}" y="{box.y / self.shrink_factor}" width="{box.width / self.shrink_factor}" height="{box.height / self.shrink_factor}" fill="{color}" />'

        assert self.plant_locator is not None
        svg = ''
        colors = {
            'person': '#ff3c3c',
            'animal': '#c5c267',
            'crop': '#00ff33',
            'charging_station': '#3bbeff',
            'rumex': '#fcfcfc',
            'thistle': '#cfddff'
        }
        for point in detections.points:
            if point.confidence < 0.3:
                continue
            color = colors.get(point.category_name, '#ff00ff')
            svg += point_to_svg(point, color=color)

        colors = {
            'car': 'yellow'
        }
        for box in detections.boxes:
            color = colors.get(box.category_name, '#ff00ff')
            svg += box_to_svg(box, color=color)
        return svg
