
import logging

import rosys

from .camera_configurations import configurations
from .simulated_cam import SimulatedCam
from .usb_cam import UsbCam


class CameraConfigurator:
    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider,
                 field_friend_version: str):
        self.log = logging.getLogger('field_friend.camera_configurator')
        self.camera_provider = camera_provider
        self.config = configurations[field_friend_version]
        rosys.on_repeat(self.update_camera_config, 10)

    def update_camera_config(self):
        for camera in self.camera_provider.cameras.values():
            if not camera.is_connected:
                continue
            if isinstance(camera, UsbCam):
                camera.set_parameters(self.config['parameters'])
            elif isinstance(camera, SimulatedCam):
                camera.resolution = rosys.vision.ImageSize(
                    width=self.config['parameters']['width'],
                    height=self.config['parameters']['height'],
                )
            if 'crop' in self.config:
                xoffset = self.config['crop']['xoffset']
                yoffset = self.config['crop']['yoffset']
                width = self.config['parameters']['width']
                height = self.config['parameters']['height']
                camera.crop = rosys.geometry.Rectangle(
                    x=xoffset,
                    y=yoffset,
                    width=width - 2 * xoffset,
                    height=height - 2 * yoffset,
                )
            if 'rotation' in self.config:
                camera.rotation = self.config['rotation']
        self.camera_provider.request_backup()
