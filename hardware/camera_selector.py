import logging

import rosys


class CameraSelector:
    def __init__(self, camera_provider: rosys.vision.CameraProvider) -> None:
        self.CAMERA_SELECTED = rosys.event.Event()
        '''A new camera was selected'''

        self.camera_provider = camera_provider
        self.camera: rosys.vision.Camera = None
        self.log = logging.getLogger('field_friend.camera_selector')
        camera_provider.CAMERA_ADDED.register(self.use_camera)

    def use_camera(self, camera: rosys.vision.Camera) -> None:
        self.camera = camera
        self.log.info(f'selected camera is {self.camera.id}')
        self.CAMERA_SELECTED.emit(self.camera)
