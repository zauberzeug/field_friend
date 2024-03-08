import rosys
from nicegui import ui


class CircleSight:

    def __init__(self, mjpg_camera_provider: rosys.vision.MjpegCameraProvider, detector: rosys.vision.DetectorHardware, *, shrink_factor: int = 1) -> None:
        self.sights: dict[str, ui.interactive_image] = {}
        self.mjpg_camera_provider = mjpg_camera_provider
        self.detector = detector
        self.person_count = 0
        self.shrink_factor = shrink_factor

    def user_interface(self):
        for camera in self.mjpg_camera_provider.cameras.values():
            self.sights[camera.id] = ui.interactive_image().classes('grow')

        async def update():
            person_count = 0
            for camera in self.mjpg_camera_provider.cameras.values():
                image = camera.latest_captured_image
                if not image:
                    continue
                if self.shrink_factor > 1:
                    source = f'{camera.get_latest_image_url()}?shrink={self.shrink_factor}'
                else:
                    source = camera.get_latest_image_url()
                self.sights[camera.id].set_source(f'{source}?shrink={self.shrink_factor}')
                await self.detector.detect(image, tags=[camera.id], autoupload=rosys.vision.Autoupload.DISABLED)
                if image.detections:
                    person_count += len([p for p in image.detections.points
                                         if p.category_name == 'person' and p.confidence > 0.4])
                    self.sights[camera.id].set_content(self.to_svg(image.detections))
            self.person_count = person_count
        ui.timer(0.5, update)

    def to_svg(self, detections: rosys.vision.Detections) -> str:
        svg = ''
        for point in detections.points:
            if point.category_name != 'person' or point.confidence < 0.4:
                continue
            svg += f'<circle cx="{point.x / self.shrink_factor}" cy="{point.y / self.shrink_factor}" r="8" fill="red" />'
        return svg
