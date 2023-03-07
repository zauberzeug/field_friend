import logging

import rosys


class CameraSelector:

    FRONT_CAMERA_IDS = ['front_cam', 'usb-70090000.xusb-2.4.1']
    BOTTOM_CAMERA_IDS = ['bottom_cam', 'usb-70090000.xusb-2.4.2']

    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        self.CAMERA_SELECTED = rosys.event.Event()
        '''A camera was selected'''

        self.camera_ids = {
            'front cam': self.FRONT_CAMERA_IDS, 'bottom cam': self.BOTTOM_CAMERA_IDS
        }
        self.camera_provider = camera_provider
        self.cameras: dict[str, rosys.vision.Camera] = {}
        self.log = logging.getLogger('field_friend.camera_selector')
        camera_provider.CAMERA_ADDED.register(self.use_camera)

    async def use_camera(self, camera: rosys.vision.Camera) -> None:
        for camera_type, camera_ids_list in self.camera_ids.items():
            if camera.id in camera_ids_list:
                self.log.info(f'camera {camera.id} is a {camera_type}')
                self.cameras[camera_type] = camera
                await rosys.sleep(1)
                self.CAMERA_SELECTED.emit((camera_type, camera))
                return
        self.log.warning(f'camera {camera.id} is not a known camera')
