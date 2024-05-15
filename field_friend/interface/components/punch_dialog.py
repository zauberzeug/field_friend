from typing import Optional

import rosys
from nicegui import ui
from nicegui.events import MouseEventArguments

from ...automations import PlantLocator


class PunchDialog(ui.dialog):
    def __init__(self, camera_provider: rosys.vision.CameraProvider, plant_locator: PlantLocator, shrink_factor: int = 1) -> None:
        super().__init__()
        self.camera: Optional[rosys.vision.CalibratableCamera] = None
        self.camera_provider = camera_provider
        self.plant_locator = plant_locator
        self.shrink_factor = shrink_factor
        self.image_view: Optional[ui.interactive_image] = None
        with self, ui.card():
            events = ['mousemove', 'mouseout', 'mouseup']
            self.image_view = ui.interactive_image(
                '',
                cross=True,
                on_mouse=self.on_mouse_move,
                events=events
            ).classes('w-full')
            self.label = ui.label('Do you want to continue the canceled automation').classes('text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.submit('Yes'))
                ui.button('No', on_click=lambda: self.submit('No'))
                ui.button('Cancel', on_click=lambda: self.submit('Cancel'))

    def open(self) -> None:
        self.update_content()
        super().open()

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
            self.camera = active_camera
        if self.shrink_factor > 1:
            url = f'{self.camera.get_latest_image_url()}?shrink={self.shrink_factor}'
        else:
            url = self.camera.get_latest_image_url()
        self.image_view.set_source(url)
        image = self.camera.latest_detected_image
        if image and image.detections:
            self.image_view.set_content(self.to_svg(image.detections))

    def to_svg(self, detections: rosys.vision.Detections) -> str:
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
        return svg

    def on_mouse_move(self, e: MouseEventArguments):
        pass
