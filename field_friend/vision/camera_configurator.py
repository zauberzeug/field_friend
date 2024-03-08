
import logging

import rosys

import config.config_selection as config_selector

from .camera_configurations import configurations
from .simulated_cam import SimulatedCam
from .usb_cam import UsbCam


class CameraConfigurator:
    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider):
        self.log = logging.getLogger('field_friend.camera_configurator')
        self.camera_provider = camera_provider
        self.config = config_selector.import_config(module='camera')
        rosys.on_repeat(self.update_camera_config, 10)

    async def update_camera_config(self):
        for camera in self.camera_provider.cameras.values():
            if not camera.is_connected:
                self.log.info(f'Not updating camera {camera.id}: not connected')
                continue
            parameters_changed = False
            if isinstance(camera, UsbCam):
                if 'u1' in list(self.config.keys()):
                    # camera.set_parameters(self.config['parameters'])
                    # Camera bug on u1 after setting the new parameters remove line and restart system
                    pass
                else:
                    # only set parameters that have changed
                    for parameter, value in self.config['parameters'].items():
                        if camera.parameters[parameter] != value:
                            camera.set_parameters({parameter: value})
                            parameters_changed = True

                if 'crop' in self.config:
                    xoffset = self.config['crop']['xoffset']
                    yoffset = self.config['crop']['yoffset']
                    width = self.config['parameters']['width']
                    height = self.config['parameters']['height']
                    crop_rectangle = rosys.geometry.Rectangle(
                        x=xoffset,
                        y=yoffset,
                        width=width - 2 * xoffset,
                        height=height - 2 * yoffset,
                    )
                    if camera.crop != crop_rectangle:
                        camera.crop = crop_rectangle
                        parameters_changed = True
                else:
                    camera.crop = None
                if 'rotation' in self.config:
                    if camera.rotation != self.config['rotation']:
                        camera.rotation = self.config['rotation']
                else:
                    camera.rotation = 0

            elif isinstance(camera, SimulatedCam):
                if camera.resolution.width != self.config['parameters']['width'] or camera.resolution.height != self.config['parameters']['height']:
                    camera.resolution = rosys.vision.ImageSize(
                        width=self.config['parameters']['width'],
                        height=self.config['parameters']['height'],
                    )
                    parameters_changed = True

            if parameters_changed:
                self.log.info(f'Updated camera {camera.id} parameters, requesting backup...')
                self.camera_provider.request_backup()
