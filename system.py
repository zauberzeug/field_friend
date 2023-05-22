import os

import rosys

from field_friend.automations import PlantProvider, Puncher, Weeding
from field_friend.hardware import FieldFriendHardware, FieldFriendSimulation
from field_friend.vision import CameraSelector
from field_friend.vision.simulation import create_weedcam


class System:
    def __init__(self) -> None:
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.camera_selector = CameraSelector()
        if self.is_real:
            self.field_friend = FieldFriendHardware()
            self.usb_camera_provider = rosys.vision.UsbCameraProviderHardware()
            self.detector = rosys.vision.DetectorHardware(port=8004)
        else:

            self.field_friend = FieldFriendSimulation()
            self.usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
        self.usb_camera_provider.CAMERA_ADDED.register(self.camera_selector.use_camera)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.2)
        self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.2
        self.driver.parameters.angular_speed_limit = 0.5
        self.driver.parameters.can_drive_backwards = False
        self.automator = rosys.automation.Automator(self.steerer, on_interrupt=self.field_friend.stop)
        self.puncher = Puncher(self.field_friend, self.driver)
        self.weeding = Weeding(self.field_friend, self.driver, self.detector,
                               self.camera_selector, self.plant_provider, self.puncher)
        self.automator.default_automation = self.weeding.start

        if not self.is_real:
            rosys.on_startup(lambda: create_weedcam('front_cam', self.usb_camera_provider))

    def restart(self) -> None:
        os.utime('main.py')
