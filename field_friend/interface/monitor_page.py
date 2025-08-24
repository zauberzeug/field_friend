import logging

import rosys
from nicegui import ui

from ..system import System
from .components import CameraCard as camera_card
from .components import KeyControls, create_header


class MonitorPage:
    """The page to monitor the robot's cameras."""

    def __init__(self, system: System) -> None:
        self.log = logging.getLogger('field_friend.monitoring')
        self.system = system
        self.plant_locator = system.plant_locator
        self.capture = system.capture
        self.automator = system.automator
        self.mjpg_camera_provider = system.mjpeg_camera_provider
        self.circle_sight_detector = system.circle_sight_detector
        self.circle_sight_active = False

        self.circle_sight_positions = system.config.circle_sight_positions
        if self.circle_sight_positions is None:
            self.log.warning('No circle sight positions configured, camera views will not be available')
        self.sights: dict[str, ui.interactive_image] = {}

        @ui.page('/monitor')
        def page() -> None:
            create_header(system)
            self._content()
            ui.timer(rosys.config.ui_update_interval, self._update_monitor_content)

    def _content(self) -> None:
        KeyControls(self.system)
        with ui.row().classes('w-full h-1/2 items-stretch items-center gap-2'):
            with ui.column().classes('w-1/6 items-center'):
                self._circle_sight_text('Front')
                self.sights['front'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                self._circle_sight_text('Left')
                self.sights['left'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                if self.circle_sight_detector is not None:
                    ui.switch('Person detection') \
                        .bind_value(self, 'circle_sight_active') \
                        .bind_enabled_from(self.automator, 'is_running', backward=lambda x: not x)

            with ui.column().classes('w-[calc(65%)] items-center'):
                with ui.row().classes('items-center'):
                    camera_card(self.system,
                                shrink_factor=1.0,
                                show_plants=True,
                                show_detections=True,
                                show_plants_to_handle=True)
                with ui.row(align_items='center'):
                    if self.circle_sight_detector is not None:
                        ui.button('Capture Outer', on_click=self.capture.circle_sight)
                        ui.button('Capture Front', on_click=lambda: self.capture.circle_sight(direction='front'))
                    if self.plant_locator is not None:
                        ui.button('Capture Inner', on_click=self.capture.inner)

            with ui.column().classes('w-1/6 items-center'):
                self._circle_sight_text('Back')
                self.sights['back'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                self._circle_sight_text('Right')
                self.sights['right'] = ui.interactive_image('assets/field_friend.webp').classes('w-full')
                with ui.row():
                    rosys.automation.automation_controls(self.automator)

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
        if not isinstance(self.circle_sight_detector, rosys.vision.DetectorHardware):
            return
        for camera in self.mjpg_camera_provider.cameras.values():
            position = camera_id_to_position(camera.id)
            if position is None:
                continue
            image = camera.latest_captured_image
            if image and self.circle_sight_active and self.circle_sight_detector.is_connected:
                await self.circle_sight_detector.detect(image, tags=[position], autoupload=rosys.vision.Autoupload.FILTERED)
                if image.detections:
                    self.sights[position].set_content(self._to_svg(image.detections))
                    continue
            self.sights[position].set_content('')

    def _to_svg(self, detections: rosys.vision.Detections) -> str:
        def point_to_svg(point: rosys.vision.PointDetection, *, color: str, radius: float = 8) -> str:
            return f'<circle cx="{point.x}" cy="{point.y}" r="{radius}" fill="{color}" />'

        def box_to_svg(box: rosys.vision.BoxDetection, *, color: str) -> str:
            return f'<rect x="{box.x}" y="{box.y}" width="{box.width}" height="{box.height}" fill="{color}" />'

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
