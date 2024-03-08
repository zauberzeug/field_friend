import rosys
from nicegui import ui

from ..automations import PlantLocator


class monitoring:

    def __init__(self,
                 usb_camera_provider: rosys.vision.CameraProvider,
                 mjpeg_camera_provider: rosys.vision.CameraProvider,
                 detector: rosys.vision.Detector,
                 monitoring_detector: rosys.vision.Detector,
                 plant_locator: PlantLocator,
                 *,
                 shrink_factor: int = 1) -> None:
        self.usb_camera_provider = usb_camera_provider
        self.mjpg_camera_provider = mjpeg_camera_provider
        self.detector = detector
        self.monitoring_detector = monitoring_detector
        self.plant_locator = plant_locator
        self.person_count = 0
        self.shrink_factor = shrink_factor

        with ui.row().classes('w-full grow gap-0'):
            column_classes = 'grow items-center mt-[200px]'
            text_style = 'font-size: 30em; line-height: 80%;'
            with ui.column().classes(column_classes):
                self.crops_count_label = ui.label().style(text_style)
                self.crops_label = ui.label('Nutzpflanzen').classes('text-2xl text-bold')
            self.bottom_view = ui.interactive_image().classes('h-[780px]')
            with ui.column().classes(column_classes):
                self.weeds_count_label = ui.label().style(text_style)
                self.weeds_label = ui.label('Beikraut').classes('text-2xl text-bold')
        with ui.row().classes('w-full items-stretch gap-0') as self.circle_sight:
            self.sights: dict[str, ui.interactive_image] = {}

        # ui.timer(0.5, self.update_monitor_content)
        # ui.timer(1, self.update_bottom_view)

    async def update_monitor_content(self):
        for camera in self.mjpg_camera_provider.cameras.values():
            if camera.id not in self.sights:
                self.sights[camera.id] = ui.interactive_image().classes('grow')
        person_count = 0
        for camera in self.mjpg_camera_provider.cameras.values():
            image = camera.latest_captured_image
            if not image:
                continue
            source = f'{camera.get_latest_image_url()}?shrink={self.shrink_factor}'
            self.sights[camera.id].set_source(f'{source}?shrink={self.shrink_factor}')
            await self.monitoring_detector.detect(image, tags=[camera.id], autoupload=rosys.vision.Autoupload.DISABLED)
            if image.detections:
                person_count += len([p for p in image.detections.points
                                     if p.category_name == 'person' and p.confidence > 0.4])
                self.sights[camera.id].set_content(self.to_svg(image.detections))
        self.person_count = person_count

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
            self.crops_count_label.set_text(crops)
            self.crops_label.set_text('Crop' if crops == 1 else 'Crops')
            self.weeds_count_label.set_text(weeds)
            self.weeds_label.set_text('Weed' if weeds == 1 else 'Weeds')
        else:
            self.weeds_count_label.set_text(0)
            self.bottom_view.set_content('')

    def to_svg(self, detections: rosys.vision.Detections) -> str:
        svg = ''
        cross_size = 20
        for point in detections.points:
            if point.category_name == 'person' and point.confidence > 0.4:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="red" />'
            elif point.category_name in self.plant_locator.crop_category_names and point.confidence > self.plant_locator.minimum_crop_confidence:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="18" stroke-width="8" stroke="green" fill="none" />'
            elif point.category_name in self.plant_locator.weed_category_names and point.confidence > self.plant_locator.minimum_weed_confidence:
                svg += f'''
                        <line x1="{point.x / self.shrink_factor - cross_size}" y1="{point.y / self.shrink_factor}" x2="{point.x / self.shrink_factor + cross_size}" y2="{point.y / self.shrink_factor}" stroke="red" stroke-width="8" 
                            transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                        <line x1="{point.x / self.shrink_factor}" y1="{point.y / self.shrink_factor - cross_size}" x2="{point.x / self.shrink_factor}" y2="{point.y / self.shrink_factor + cross_size}" stroke="red" stroke-width="8" 
                            transform="rotate(45, {point.x / self.shrink_factor}, {point.y / self.shrink_factor})"/>
                '''
            else:
                svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="blue" />'
        return svg
