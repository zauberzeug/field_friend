import os

import rosys

from field_friend.automations import PlantDetector, PlantProvider, Puncher, Weeding
from field_friend.hardware import FieldFriendHardware, FieldFriendSimulation
from field_friend.navigation import GnssHardware, GnssSimulation
from field_friend.vision import CameraSelector
from field_friend.vision.simulation import create_weedcam


class System:
    def __init__(self) -> None:
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.camera_selector = CameraSelector()
        self.camera_selector.camera_ids = {
            'bottom cam': self.camera_selector.BOTTOM_CAMERA_IDS,
        }
        if self.is_real:
            self.field_friend = FieldFriendHardware()
            self.usb_camera_provider = rosys.vision.UsbCameraProviderHardware()
            self.detector = rosys.vision.DetectorHardware(port=8004)
            self.gnss = GnssHardware()
        else:

            self.field_friend = FieldFriendSimulation()
            self.usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
            self.gnss = GnssSimulation(self.field_friend.wheels)
        self.usb_camera_provider.CAMERA_ADDED.register(self.camera_selector.use_camera)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.2)
        self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
        self.gnss.ROBOT_LOCATED.register(self.odometer.handle_detection)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.4
        self.driver.parameters.angular_speed_limit = 0.4
        self.driver.parameters.can_drive_backwards = False
        self.driver.parameters.minimum_turning_radius = 1.0
        self.automator = rosys.automation.Automator(self.steerer, on_interrupt=self.field_friend.stop)
        self.puncher = Puncher(self.field_friend, self.driver)
        self.plant_detector = PlantDetector(self.detector, self.plant_provider, self.odometer)
        self.weeding = Weeding(self.field_friend, self.driver, self.detector,
                               self.camera_selector, self.plant_provider, self.puncher, self.plant_detector)
        self.automator.default_automation = self.weeding.start

        if self.is_real:
            rosys.automation.app_controls(self.field_friend.robot_brain, self.automator)

            def configure_cameras() -> None:
                for camera in self.usb_camera_provider.cameras.values():
                    width, height, xoffset, yoffset = 1920, 1080, 420, 150
                    camera.resolution = rosys.vision.ImageSize(width=width, height=height)
                    camera.crop = rosys.geometry.Rectangle(
                        x=xoffset, y=0, width=width - (2 * xoffset + 150),
                        height=height - yoffset)
                    # camera.crop = rosys.geometry.Rectangle(
                    #     x=0, y=0, width=width,
                    #     height=height)
                    camera.auto_exposure = True
            rosys.on_repeat(configure_cameras, 1.0)

            self.was_charging = False

            def check_if_charging():
                if self.field_friend.bms.state.is_charging:
                    self.was_charging = True
                    return
                if not self.field_friend.bms.state.is_charging and self.was_charging:
                    self.automator.start(self.field_friend.battery_control.release_battery_relais())
                    self.was_charging = False

            def relase_relais_on_startup():
                self.automator.start(self.field_friend.battery_control.release_battery_relais())
            rosys.on_repeat(check_if_charging, 0.5)
            rosys.on_startup(relase_relais_on_startup)
        else:
            rosys.on_startup(lambda: create_weedcam('bottom_cam', self.usb_camera_provider))

        def stop():
            if self.automator.is_running:
                self.automator.stop(because='emergency stop triggered')
        self.field_friend.estop.ESTOP_TRIGGERED.register(stop)

    def restart(self) -> None:
        os.utime('main.py')
