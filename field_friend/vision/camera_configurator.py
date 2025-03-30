
import logging

import rosys

from config import CameraConfiguration

from ..robot_locator import RobotLocator
from .calibratable_usb_camera import CalibratableUsbCamera


class CameraConfigurator:
    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider,
                 robot_locator: RobotLocator,
                 robot_id: str,
                 camera_config: CameraConfiguration,
                 ):
        self.log = logging.getLogger('field_friend.camera_configurator')
        self.camera_provider = camera_provider
        self.robot_locator = robot_locator
        self.robot_id = robot_id
        self.config = camera_config
        rosys.on_startup(self.update_camera_config)

    async def update_camera_config(self):
        await rosys.sleep(15)
        self.log.debug('updating camera config')
        camera = None
        start_time = rosys.time()
        while not camera:
            if rosys.time() - start_time > 60:
                raise RuntimeError('No active camera found after 60s')
            cameras = list(self.camera_provider.cameras.values())
            camera = next((camera for camera in cameras if camera.is_connected), None)
            await rosys.sleep(1)
        self.log.debug(f'camera: {camera.id} is active')
        parameters_changed = False
        if isinstance(camera, CalibratableUsbCamera):
            if self.robot_id == 'u1':
                # camera.set_parameters(self.config['parameters'])
                # !Camera bug on u1 after setting the new parameters remove line and restart system
                pass
            else:
                # only set parameters that have changed
                for name, value in self.config.parameters.items():
                    self.log.info(f'parameter {name} with value {value}')
                    if camera.parameters[name] != value:
                        self.log.info(f'{camera.parameters[name]} != {value}')
                        await camera.set_parameters({name: value})
                        parameters_changed = True
            if camera.calibration is not None and not camera.calibration.extrinsics.frame_id:
                camera.calibration.extrinsics.in_frame(self.robot_locator.pose_frame)
            if self.config.crop:
                crop_rectangle = self.config.crop_rectangle
                if camera.crop != crop_rectangle:
                    camera.crop = crop_rectangle
                    parameters_changed = True
            else:
                camera.crop = None
            if camera.rotation_angle != self.config.rotation:
                camera.rotation_angle += self.config.rotation
                parameters_changed = True
                self.log.info(f'camera rotation: {camera.rotation}; {camera.rotation_angle}')
            else:
                camera.rotation_angle = 0

        elif isinstance(camera, rosys.vision.SimulatedCalibratableCamera):
            if camera.resolution.width != self.config.width or camera.resolution.height != self.config.height:
                camera.resolution = rosys.vision.ImageSize(
                    width=self.config.width,
                    height=self.config.height,
                )
                parameters_changed = True

        if parameters_changed:
            self.log.debug(f'Updated camera {camera.id} parameters, requesting backup...')
            self.camera_provider.request_backup()
