from typing import Optional

import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments
from rosys.geometry import Point3d
from rosys.vision import Image

from ...automations import Plant, PlantLocator


class PunchDialog(ui.dialog):
    def __init__(self, camera_provider: rosys.vision.CameraProvider, plant_locator: PlantLocator, shrink_factor: int = 1) -> None:
        super().__init__()
        self.camera: Optional[rosys.vision.CalibratableCamera] = None
        self.camera_provider = camera_provider
        self.plant_locator = plant_locator
        self.shrink_factor = shrink_factor
        self.static_image_view: Optional[ui.interactive_image] = None
        self.live_image_view: Optional[ui.interactive_image] = None
        self.target_plant: Optional[Plant] = None
        self.timer = ui.timer(0.2, self.update_live_view, active=False)
        self.setup_camera()
        with self, ui.card().style('max-width: 1400px'):
            with ui.row(wrap=False):
                with ui.column().classes('w-1/2'):
                    ui.label('Last detection').classes('text-lg')
                    self.static_image_view = ui.interactive_image('')
                with ui.column().classes('w-1/2'):
                    ui.label('Live').classes('text-lg')
                    self.live_image_view = ui.interactive_image('')
            self.label = ui.label('Do you want to continue the canceled automation').classes('text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.submit('Yes'))
                ui.button('No', on_click=lambda: self.submit('No'))
                ui.button('Cancel', on_click=lambda: self.submit('Cancel'))

    def submit(self, value: str) -> None:
        self.timer.active = False
        super().submit(value)

    def open(self) -> None:
        assert self.target_plant is not None
        assert self.camera is not None
        detection_image = self.camera.latest_detected_image if self.target_plant.detection_image is None else self.target_plant.detection_image
        self.update_content(self.static_image_view, detection_image, draw_target=True)
        self.timer.active = True
        super().open()

    def setup_camera(self) -> None:
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
            self.camera = active_camera

    def update_content(self, image_view: ui.interactive_image, image: Image, draw_target: bool = False) -> None:
        assert self.camera is not None
        if self.shrink_factor > 1:
            url = f'{self.camera.get_latest_image_url()}?shrink={self.shrink_factor}'
        else:
            url = self.camera.get_latest_image_url()
        image_view.set_source(url)
        if image and image.detections:
            target_point = None
            if self.target_plant and draw_target:
                target_point = self.camera.calibration.project_to_image(
                    Point3d(x=self.target_plant.position.x, y=self.target_plant.position.y, z=0))
            image_view.set_content(self.to_svg(image.detections, target_point))

    def update_live_view(self) -> None:
        assert self.camera is not None
        self.update_content(self.live_image_view, self.camera.latest_detected_image)

    def to_svg(self, detections: rosys.vision.Detections, target_point: Optional[rosys.geometry.Point]) -> str:
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
        if target_point:
            svg += f'''<circle cx="{target_point.x / self.shrink_factor}" cy="{target_point.y /
                                                                               self.shrink_factor}" r="18" stroke-width="8" stroke="gold" fill="none" />'''
        return svg