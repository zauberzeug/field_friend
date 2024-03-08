import logging
import os

import numpy as np
import rosys
from rosys.analysis import KpiLogger

from field_friend.automations import (AutomationWatcher, BatteryWatcher, CoinCollecting, FieldProvider, Mowing,
                                      PathProvider, PathRecorder, PlantLocator, PlantProvider, Puncher, Weeding)
from field_friend.hardware import FieldFriendHardware, FieldFriendSimulation
from field_friend.navigation import GnssHardware, GnssSimulation
from field_friend.vision import CameraConfigurator, SimulatedCam, SimulatedCamProvider, UsbCamProvider

from .interface.info import Info
from .kpi_generator import generate_kpis


class System:
    def __init__(self) -> None:
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.version = 'u4'  # insert here your field friend version
        if self.is_real:
            self.field_friend = FieldFriendHardware(version=self.version)
            self.usb_camera_provider = UsbCamProvider()
            self.detector = rosys.vision.DetectorHardware(port=8004)
            # self.circle_sight = CircleSight()
        else:
            self.field_friend = FieldFriendSimulation(version=self.version)
            self.usb_camera_provider = SimulatedCamProvider()
            self.usb_camera_provider.remove_all_cameras()
            self.usb_camera_provider.add_camera(SimulatedCam.create_calibrated(id='bottom_cam',
                                                                               x=0.4, z=0.4,
                                                                               roll=np.deg2rad(360-150),
                                                                               pitch=np.deg2rad(0),
                                                                               yaw=np.deg2rad(90)))
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
            # self.circle_sight = None
        self.camera_configurator = CameraConfigurator(self.usb_camera_provider, self.version)
        self.plant_provider = PlantProvider()
        self.field_provider = FieldProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.25)
        self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
        if self.is_real:
            self.gnss = GnssHardware(self.odometer)
        else:
            self.gnss = GnssSimulation(self.field_friend.wheels)
        self.gnss.ROBOT_POSE_LOCATED.register(self.forward_pose_odometer)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.1
        self.driver.parameters.angular_speed_limit = 0.5
        self.driver.parameters.can_drive_backwards = True
        self.driver.parameters.minimum_turning_radius = 0.05
        self.driver.parameters.hook_offset = 0.6
        self.driver.parameters.carrot_distance = 0.2
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
        self.puncher = Puncher(self.field_friend, self.driver)

        self.kpi_logger = KpiLogger()
        if not (self.is_real):
            generate_kpis(self.kpi_logger)

        def watch_robot() -> None:
            self.kpi_logger.increment_on_rising_edge('bumps', bool(self.field_friend.bumper.active_bumpers))
            self.kpi_logger.increment_on_rising_edge('low_battery', self.field_friend.bms.is_below_percent(10.0))
            # TODO add more things to watch after

        self.big_weed_category_names = ['thistle', 'big_weed', 'orache']
        self.small_weed_category_names = ['weed', 'coin']
        self.crop_category_names = ['sugar_beet', 'crop', 'coin_with_hole']
        self.plant_locator = PlantLocator(self.usb_camera_provider, self.detector,
                                          self.plant_provider, self.odometer, self.kpi_logger)
        self.plant_locator.weed_category_names = self.big_weed_category_names + self.small_weed_category_names
        self.plant_locator.crop_category_names = self.crop_category_names

        rosys.on_repeat(watch_robot, 1.0)
        # TODO außerdem loggen wenn etwas in den Automationen passiert (überlegen wo dann genau geloggt wird)

        self.weeding = Weeding(self)
        self.coin_collecting = CoinCollecting(self)
        self.path_provider = PathProvider()
        self.path_recorder = PathRecorder(self.path_provider, self.driver, self.steerer, self.gnss)
        self.field_provider = FieldProvider()
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
        self.mowing = Mowing(self.field_friend, self.field_provider, driver=self.driver,
                             path_planner=self.path_planner, gnss=self.gnss, robot_width=width, kpi_logger=self.kpi_logger)

        self.automations = {
            'weeding': self.weeding.start,
            'mowing': self.mowing.start,
            'collecting (demo)': self.coin_collecting.start,
        }
        self.automator = rosys.automation.Automator(None, on_interrupt=self.field_friend.stop,
                                                    default_automation=self.coin_collecting.start)
        self.info = Info(self)
        if self.field_friend.bumper is not None:
            self.automation_watcher = AutomationWatcher(self.automator, self.odometer, self.field_friend.bumper)
        if self.is_real:
            if self.field_friend.battery_control:
                self.battery_watcher = BatteryWatcher(self.field_friend, self.automator)
            rosys.automation.app_controls(self.field_friend.robot_brain, self.automator)

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

    def forward_pose_odometer(self, pose: rosys.geometry.Pose) -> None:
        self.odometer.handle_detection(pose)

    def restart(self) -> None:
        os.utime('main.py')
