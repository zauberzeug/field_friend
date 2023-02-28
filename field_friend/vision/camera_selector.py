import logging

import rosys

FRONT_CAMERA_IDS = ['front_cam', 'usb-70090000.xusb-2.4.2']
BOTTOM_CAMERA_IDS = ['bottom_cam', 'usb-70090000.xusb-2.4.1']


class CameraSelector:
    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        self.FRONT_CAMERA_SELECTED = rosys.event.Event()
        '''A new front camera was selected'''
        self.BOTTOM_CAMERA_SELECTED = rosys.event.Event()
        '''A new bottom camera was selected'''

        self.camera_provider = camera_provider
        self.camera: rosys.vision.Camera = None
        self.log = logging.getLogger('field_friend.camera_selector')
        camera_provider.CAMERA_ADDED.register(self.use_camera)

    def use_camera(self, camera: rosys.vision.Camera) -> None:
        self.camera = camera
        self.log.info(f'selected camera is {self.camera.id}')
        if camera.id not in FRONT_CAMERA_IDS and camera.id not in BOTTOM_CAMERA_IDS:
            self.log.warning(f'camera {camera.id} is not a known camera')
        if camera.id in FRONT_CAMERA_IDS:
            self.log.info(f'camera {camera.id} is a front camera')
            self.FRONT_CAMERA_SELECTED.emit(self.camera)
        if camera.id in BOTTOM_CAMERA_IDS:
            self.log.info(f'camera {camera.id} is a bottom camera')
            self.BOTTOM_CAMERA_SELECTED.emit(self.camera)
