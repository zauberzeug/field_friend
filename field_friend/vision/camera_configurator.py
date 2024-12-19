
import logging

import rosys

import config.config_selection as config_selector

from ..robot_locator import RobotLocator
from .calibratable_usb_camera import CalibratableUsbCamera


class CameraConfigurator:
    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider,
                 robot_locator: RobotLocator,
                 robot_id: str | None = None,
                 ):
        self.log = logging.getLogger('field_friend.camera_configurator')
        self.camera_provider = camera_provider
        self.robot_locator = robot_locator
        if not robot_id:
            self.config = config_selector.import_config(module='camera')
        else:
            self.config = config_selector.import_config_simulation(module='camera', robot_id=robot_id)
        rosys.on_startup(self.update_camera_config)

    async def update_camera_config(self):
        await rosys.sleep(15)
        self.log.info('updating camera config')
        camera = None
        start_time = rosys.time()
        while not camera:
            if rosys.time() - start_time > 60:
                raise RuntimeError('No active camera found after 60s')
            cameras = list(self.camera_provider.cameras.values())
            camera = next((camera for camera in cameras if camera.is_connected), None)
            await rosys.sleep(1)
        self.log.info(f'camera: {camera.id} is active')
        parameters_changed = False
        if isinstance(camera, CalibratableUsbCamera):
            if 'u1' in list(self.config.keys()):
                # camera.set_parameters(self.config['parameters'])
                # !Camera bug on u1 after setting the new parameters remove line and restart system
                pass
            else:
                # only set parameters that have changed
                for name, value in self.config['parameters'].items():
                    self.log.info(f'parameter {name} with value {value}')
                    if camera.parameters[name] != value:
                        self.log.info(f'{camera.parameters[name]} != {value}')
                        await camera.set_parameters({name: value})
                        parameters_changed = True
            if not camera.calibration.extrinsics.frame_id:
                camera.calibration.extrinsics.in_frame(self.robot_locator.pose_frame)
            if 'crop' in self.config:
                # Fetch new cropping parameters
                left = self.config['crop']['left']
                right = self.config['crop']['right']
                up = self.config['crop']['up']
                down = self.config['crop']['down']

                # Calculate the new width and height after cropping
                total_width = self.config['parameters']['width']
                total_height = self.config['parameters']['height']
                new_width = total_width - (left + right)
                new_height = total_height - (up + down)

                # Create the new crop rectangle
                crop_rectangle = rosys.geometry.Rectangle(
                    x=left,
                    y=up,
                    width=new_width,
                    height=new_height,
                )

                # Update the camera crop if necessary
                if camera.crop != crop_rectangle:
                    camera.crop = crop_rectangle
                    parameters_changed = True
            else:
                camera.crop = None
            if 'rotation' in self.config:
                if camera.rotation_angle != self.config['rotation']:
                    camera.rotation_angle += self.config['rotation']
                    parameters_changed = True
                    self.log.info(f'camera rotation: {camera.rotation}; {camera.rotation_angle}')
            else:
                camera.rotation_angle = 0

        elif isinstance(camera, rosys.vision.SimulatedCalibratableCamera):
            if camera.resolution.width != self.config['parameters']['width'] or camera.resolution.height != self.config['parameters']['height']:
                camera.resolution = rosys.vision.ImageSize(
                    width=self.config['parameters']['width'],
                    height=self.config['parameters']['height'],
                )
                parameters_changed = True

        if parameters_changed:
            self.log.info(f'Updated camera {camera.id} parameters, requesting backup...')
            self.camera_provider.request_backup()
