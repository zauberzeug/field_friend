import rosys
from nicegui import ui


class CircleSight:

    SHRINK = 2

    def __init__(self):
        self.mjpg_camera_provider = rosys.vision.MJpegCameraProviderHardware()
        rosys.background_tasks.create(self.create_cameras())
        self.sights: dict[str, ui.interactive_image] = {}
        self.detector = rosys.vision.DetectorHardware(port=8005)
        self.person_count = 0

    async def create_cameras(self):
        self.mjpg_camera_provider.cameras.clear()
        base_url = 'http://192.168.168.105/axis-cgi/mjpg/video.cgi?camera='
        await self.mjpg_camera_provider.create_camera('front', base_url + '3')
        await self.mjpg_camera_provider.create_camera('right', base_url + '1')
        await self.mjpg_camera_provider.create_camera('back', base_url + '2')
        await self.mjpg_camera_provider.create_camera('left', base_url + '4')

    def user_interface(self):
        for camera in self.mjpg_camera_provider.cameras.values():
            self.sights[camera.id] = ui.interactive_image().classes('grow')

        async def update():
            person_count = 0
            for camera in self.mjpg_camera_provider.cameras.values():
                image = camera.latest_captured_image
                if not image:
                    continue
                source = self.mjpg_camera_provider.get_image_url(image)
                self.sights[camera.id].set_source(f'{source}?shrink={self.SHRINK}')
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
            svg += f'<circle cx="{point.x / self.SHRINK}" cy="{point.y / self.SHRINK}" r="8" fill="red" />'
        return svg
