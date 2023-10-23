import os

import rosys

from field_friend.automations import (FieldProvider, Mowing, PathProvider, PathRecorder, PlantDetector, PlantProvider,
                                      Puncher, Weeding)
from field_friend.hardware import FieldFriendHardware, FieldFriendSimulation
from field_friend.navigation import GnssHardware, GnssSimulation
from field_friend.vision import CameraSelector
from field_friend.vision.simulation import create_weedcam


class System:
    def __init__(self) -> None:
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        version = 'ff3'  # insert here your field friend version
        self.camera_selector = CameraSelector()
        self.camera_selector.camera_ids = {
            'bottom cam': self.camera_selector.BOTTOM_CAMERA_IDS,
        }
        if self.is_real:
            self.field_friend = FieldFriendHardware(version=version)
            self.usb_camera_provider = rosys.vision.UsbCameraProviderHardware()
            self.detector = rosys.vision.DetectorHardware(port=8004)
        else:

            self.field_friend = FieldFriendSimulation(version=version)
            self.usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
        self.usb_camera_provider.CAMERA_ADDED.register(self.camera_selector.use_camera)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.25)
        self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
        if self.is_real:
            self.gnss = GnssHardware(self.odometer)
        else:
            self.gnss = GnssSimulation(self.field_friend.wheels)
        self.gnss.ROBOT_LOCATED.register(self.odometer.handle_detection)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.5
        self.driver.parameters.angular_speed_limit = 1.0
        self.driver.parameters.can_drive_backwards = False
        self.driver.parameters.minimum_turning_radius = 1.0
        self.driver.parameters.hook_offset = 0.6
        self.driver.parameters.carrot_distance = 0.2
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
        self.automator = rosys.automation.Automator(steerer=None, on_interrupt=self.field_friend.stop)
        if self.is_real:
            rosys.automation.app_controls(self.field_friend.robot_brain, self.automator)
        self.puncher = Puncher(self.field_friend, self.driver)
        self.plant_detector = PlantDetector(self.detector, self.plant_provider, self.odometer)
        self.plant_detector.weed_category_names = ['coin', 'weed']
        self.plant_detector.crop_category_names = ['sugar_beet', 'crop']
        self.plant_detector.minimum_crop_confidence = 0.5
        self.plant_detector.minimum_weed_confidence = 0.5
        self.weeding = Weeding(self.field_friend, self.driver, self.detector,
                               self.camera_selector, self.plant_provider, self.puncher, self.plant_detector)

        self.path_provider = PathProvider()
        self.path_recorder = PathRecorder(self.path_provider, self.driver, self.steerer, self.gnss)

        width = 0.64
        length = 0.78
        offset = 0.36
        height = 0.67
        self.shape = rosys.geometry.Prism(
            outline=[
                (-offset, -width/2),
                (length - offset, -width/2),
                (length - offset, width/2),
                (-offset, width/2)
            ],
            height=height)
        self.path_planner = rosys.pathplanning.PathPlanner(self.shape)
        self.field_provider = FieldProvider()
        self.mowing = Mowing(self.field_friend, self.field_provider, driver=self.driver,
                             path_planner=self.path_planner, gnss=self.gnss, robot_width=width,)

        self.automations = {'weeding': self.weeding.start, 'mowing': self.mowing.start}
        self.automator.default_automation = self.weeding.start

        if self.is_real:
            # camera configuration
            def configure_cameras() -> None:
                if self.field_friend.version == 'ff3':
                    for camera in self.usb_camera_provider.cameras.values():
                        width, height, xoffset, yoffset = 1920, 1080, 420, 150
                        camera.resolution = rosys.vision.ImageSize(width=width, height=height)
                        camera.crop = rosys.geometry.Rectangle(
                            x=xoffset, y=0, width=width - (2 * xoffset + 150),
                            height=height - yoffset)
                        camera.auto_exposure = True
                else:
                    for camera in self.usb_camera_provider.cameras.values():
                        width, height, xoffset, yoffset = 1920, 1080, 450, 30
                        camera.resolution = rosys.vision.ImageSize(width=width, height=height)
                        camera.crop = rosys.geometry.Rectangle(
                            x=xoffset, y=yoffset, width=width - 2 * xoffset, height=height - 2 * yoffset)
                        camera.auto_exposure = False
                        camera.exposure = 0.002

            rosys.on_repeat(configure_cameras, 1.0)

            if self.field_friend.battery_control is not None:
                # battery control
                def check_if_charging():
                    if self.field_friend.bms.state.is_charging:
                        self.was_charging = True
                        return
                    if not self.field_friend.bms.state.is_charging and self.was_charging:
                        self.automator.start(self.field_friend.battery_control.release_battery_relay())
                        self.was_charging = False

                def relase_relais_on_startup():
                    self.automator.start(self.field_friend.battery_control.release_battery_relay())

                self.was_charging = False
                rosys.on_repeat(check_if_charging, 0.5)
                rosys.on_startup(relase_relais_on_startup)

        else:
            rosys.on_startup(lambda: create_weedcam('bottom_cam', self.usb_camera_provider))

        async def stop():
            if self.automator.is_running:
                if self.field_friend.estop.is_soft_estop_active:
                    self.automator.pause(because='soft estop active')
                else:
                    self.automator.pause(because='emergency stop triggered')
            await self.field_friend.stop()

        def pause():
            if self.automator.is_running:
                if self.path_recorder.state != 'recording':
                    self.automator.pause(because='steering started')

        self.steerer.STEERING_STARTED.register(pause)
        self.field_friend.estop.ESTOP_TRIGGERED.register(stop)

    def restart(self) -> None:
        os.utime('main.py')
