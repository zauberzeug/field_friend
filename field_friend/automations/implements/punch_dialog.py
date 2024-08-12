from typing import TYPE_CHECKING, Optional

import rosys
from nicegui import ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Odometer
from rosys.geometry import Point3d
from rosys.vision import Image

from ..plant import Plant

if TYPE_CHECKING:
    from field_friend.system import System

    from ...automations import PlantLocator


class PunchDialog(ui.dialog):
    def __init__(self, system: 'System', shrink_factor: int = 1, timeout: float = 20.0, ui_update_rate: float = 0.2) -> None:
        super().__init__()
        self.camera_provider = system.camera_provider
        self.plant_locator: 'PlantLocator' = system.plant_locator
        self.odometer: Odometer = system.odometer
        self.automator: Automator = system.automator
        self.shrink_factor: int = shrink_factor
        self.timeout: float = timeout
        self._duration_left: float = timeout
        self.ui_update_rate: float = ui_update_rate

        self.camera: Optional[rosys.vision.CalibratableCamera] = None
        self.static_image_view: Optional[ui.interactive_image] = None
        self.live_image_view: Optional[ui.interactive_image] = None
        self.target_plant: Optional[Plant] = None
        self.timer = ui.timer(self.ui_update_rate, self.update_live_view)
        self.setup_camera()
        with self, ui.card().style('max-width: 1400px'):
            with ui.row(wrap=False):
                with ui.column().classes('w-1/2'):
                    ui.label('Last detection').classes('text-lg')
                    self.static_image_view = ui.interactive_image('')
                with ui.column().classes('w-1/2'):
                    ui.label('Live').classes('text-lg')
                    self.live_image_view = ui.interactive_image('')
            self.label = ui.label('Do you want to punch at the current position?').classes('text-lg')
            with ui.row().classes('w-full'):
                # TODO: doesn't load properly on first open
                ui.circular_progress(min=0.0, max=self.timeout,
                                     show_value=False, size='lg').bind_value_from(self, '_duration_left')
                ui.button('Yes', on_click=lambda: self.submit('Yes'))
                ui.button('No', on_click=lambda: self.submit('No'))
                ui.space()
                automation_controls(self.automator)

    def open(self) -> None:
        self._duration_left = self.timeout
        assert self.target_plant is not None
        assert self.camera is not None
        detection_image = self.camera.latest_detected_image if self.target_plant.detection_image is None else self.target_plant.detection_image
        assert detection_image is not None
        assert self.static_image_view is not None
        self.update_content(self.static_image_view, detection_image, draw_target=True)
        self.timer.activate()
        super().open()

    def close(self) -> None:
        self.timer.deactivate()
        super().close()

    def setup_camera(self) -> None:
        cameras = list(self.camera_provider.cameras.values())
        active_camera = next((camera for camera in cameras if camera.is_connected), None)
        assert active_camera is not None
        assert isinstance(active_camera, rosys.vision.CalibratableCamera)
        self.camera = active_camera

    def update_content(self, image_view: ui.interactive_image, image: Image, draw_target: bool = False) -> None:
        self._duration_left -= self.ui_update_rate
        assert self.camera is not None
        if self.shrink_factor > 1:
            url = f'{self.camera.get_latest_image_url()}?shrink={self.shrink_factor}'
        else:
            url = self.camera.get_latest_image_url()
        image_view.set_source(url)
        if image and image.detections:
            target_point = None
            confidence = None
            assert self.camera.calibration is not None
            if self.target_plant and draw_target:
                confidence = self.target_plant.confidence
                point = self.target_plant.position
                target_point = self.camera.calibration.project_to_image(Point3d(x=point.x, y=point.y, z=0))
            image_view.set_content(self.to_svg(image.detections, target_point, confidence))

    def update_live_view(self) -> None:
        assert self.live_image_view
        if self.camera is None:
            self.live_image_view.set_source('assets/field_friend.webp')
            return
        if self.camera.latest_detected_image is not None:
            self.update_content(self.live_image_view, self.camera.latest_detected_image)

    def to_svg(self, detections: rosys.vision.Detections, target_point: Optional[rosys.geometry.Point], confidence: Optional[float], color: str = 'blue') -> str:
        svg = ''
        cross_size = 20
        for point in detections.points:
            if point.category_name in self.plant_locator.crop_category_names:
                if point.confidence > self.plant_locator.minimum_crop_confidence:
                    svg += f'''<circle cx="{point.x / self.shrink_factor}" cy="{point.y /
                                                                                self.shrink_factor}" r="18" stroke-width="8" stroke="green" fill="none" />'''
                    svg += f'''<text x="{point.x / self.shrink_factor-30}" y="{point.y /
                                                                               self.shrink_factor+30}" font-size="20" fill="green">Crop</text>'''
                else:
                    svg += f'''<circle cx="{point.x / self.shrink_factor}" cy="{point.y /
                                                                                self.shrink_factor}" r="18" stroke-width="8" stroke="yellow" fill="none" />'''
                    svg += f'''<text x="{point.x / self.shrink_factor-30}" y="{point.y /
                                                                               self.shrink_factor+30}" font-size="20" fill="yellow">{point.category_name}</text>'''
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
        if target_point and confidence:
            svg += f'''<circle cx="{target_point.x / self.shrink_factor}" cy="{target_point.y /
                                                                               self.shrink_factor}" r="20" stroke-width="8" stroke="{color}" fill="none" />'''
            svg += f'''<text x="{target_point.x / self.shrink_factor+20}" y="{target_point.y /
                                                                              self.shrink_factor-20}" font-size="20" fill="{color}">{confidence}</text>'''
        return svg
