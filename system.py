import os

import rosys

from field_friend.automations import PlantProvider, Weeding
from field_friend.hardware import CameraSelector, RobotHardware, RobotSimulation
from field_friend.hardware.simulation import create_weedcam


class System:
    def __init__(self) -> None:
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        if self.is_real:
            self.communication = rosys.hardware.SerialCommunication()
            self.robot_brain = rosys.hardware.RobotBrain(self.communication)
            if self.communication.device_path == '/dev/ttyTHS0':
                self.robot_brain.lizard_firmware.flash_params = ['xavier']
            self.robot = RobotHardware(self.robot_brain)
            self.usb_camera_provider = rosys.vision.UsbCameraProviderHardware()
            self.detector = rosys.vision.DetectorHardware(port=8004)
        else:
            self.robot = RobotSimulation()
            self.usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
        self.camera_selector = CameraSelector(self.usb_camera_provider)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.robot, speed_scaling=0.2)
        self.odometer = rosys.driving.Odometer(self.robot)
        self.driver = rosys.driving.Driver(self.robot, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.2
        self.driver.parameters.angular_speed_limit = 0.5
        self.driver.parameters.can_drive_backwards = False
        self.automator = rosys.automation.Automator(self.robot, self.steerer)
        self.weeding = Weeding(self.robot, self.driver, self.detector, self.camera_selector, self.plant_provider)
        self.automator.default_automation = self.weeding.start

        if not self.is_real:
            rosys.on_startup(lambda: create_weedcam(self.usb_camera_provider))

    def restart(self) -> None:
        os.utime('main.py')
