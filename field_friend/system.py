import logging
import os
from typing import Any

import numpy as np
import rosys

from field_friend.hardware import FieldFriend, FieldFriendHardware, FieldFriendSimulation
from field_friend.navigation.gnss_hardware import GnssHardware
from field_friend.navigation.gnss_simulation import GnssSimulation
from field_friend.vision import CalibratableUsbCameraProvider, CameraConfigurator, SimulatedCam, SimulatedCamProvider

from .automations import (AutomationWatcher, BatteryWatcher, CoinCollecting, FieldProvider, KpiProvider, Mowing,
                          PathProvider, PathRecorder, PlantLocator, PlantProvider, Puncher)
from .automations.implements import ChopAndScrew, Implement, Recorder, Tornado, WeedingScrew
from .automations.navigation import FieldNavigation, FollowCropsNavigation, Navigation, StraightLineNavigation
from .interface.components.info import Info
from .kpi_generator import generate_kpis


class System(rosys.persistence.PersistentModule):

    version = 'rb34'  # insert here your field friend version to be simulated

    def __init__(self) -> None:
        super().__init__()
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.AUTOMATION_CHANGED = rosys.event.Event()

        self.usb_camera_provider: CalibratableUsbCameraProvider | SimulatedCamProvider
        self.detector: rosys.vision.DetectorHardware | rosys.vision.DetectorSimulation
        self.field_friend: FieldFriend
        if self.is_real:
            self.field_friend = FieldFriendHardware()
            self.usb_camera_provider = CalibratableUsbCameraProvider()
            self.mjpeg_camera_provider = rosys.vision.MjpegCameraProvider(username='root', password='zauberzg!')
            self.detector = rosys.vision.DetectorHardware(port=8004)
            self.monitoring_detector = rosys.vision.DetectorHardware(port=8005)
            self.camera_configurator = CameraConfigurator(self.usb_camera_provider)
        else:
            self.field_friend = FieldFriendSimulation(robot_id=self.version)
            self.usb_camera_provider = SimulatedCamProvider()
            # NOTE we run this in rosys.startup to enforce setup AFTER the persistence is loaded
            rosys.on_startup(self.setup_simulated_usb_camera)
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
            self.camera_configurator = CameraConfigurator(self.usb_camera_provider, robot_id=self.version)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.25)
        self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
        self.gnss: GnssHardware | GnssSimulation
        if self.is_real:
            assert isinstance(self.field_friend, FieldFriendHardware)
            self.gnss = GnssHardware(self.odometer, self.field_friend.ANTENNA_OFFSET)
        else:
            self.gnss = GnssSimulation(self.odometer)
        self.gnss.ROBOT_POSE_LOCATED.register(self.odometer.handle_detection)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.3
        self.driver.parameters.angular_speed_limit = 0.8
        self.driver.parameters.can_drive_backwards = True
        self.driver.parameters.minimum_turning_radius = 0.01
        self.driver.parameters.hook_offset = 0.45
        self.driver.parameters.carrot_distance = 0.15
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance

        self.kpi_provider = KpiProvider(self.plant_provider)
        if not self.is_real:
            generate_kpis(self.kpi_provider)

        def watch_robot() -> None:
            if self.field_friend.bumper:
                self.kpi_provider.increment_on_rising_edge('bumps', bool(self.field_friend.bumper.active_bumpers))
            if self.field_friend.bms:
                self.kpi_provider.increment_on_rising_edge('low_battery', self.field_friend.bms.is_below_percent(10.0))

        self.puncher = Puncher(self.field_friend, self.driver, self.kpi_provider)
        self.big_weed_category_names = ['big_weed', 'thistle', 'orache',]
        self.small_weed_category_names = ['coin', 'weed',]
        self.crop_category_names = ['coin_with_hole', 'crop', 'sugar_beet', 'onion', 'garlic', 'maize', ]
        self.plant_locator = PlantLocator(self.usb_camera_provider,
                                          self.detector,
                                          self.plant_provider,
                                          self.odometer,
                                          )
        self.plant_locator.weed_category_names = self.big_weed_category_names + self.small_weed_category_names
        self.plant_locator.crop_category_names = self.crop_category_names
        if self.field_friend.implement_name == 'tornado':
            self.plant_locator.minimum_weed_confidence = 0.8
            self.plant_locator.minimum_crop_confidence = 0.75
        else:
            self.plant_locator.minimum_weed_confidence = 0.45
            self.plant_locator.minimum_crop_confidence = 0.65

        rosys.on_repeat(watch_robot, 1.0)

        self.path_provider = PathProvider()
        self.field_provider = FieldProvider(self.gnss)
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
        self.automator = rosys.automation.Automator(None, on_interrupt=self.field_friend.stop)
        self.automation_watcher = AutomationWatcher(self)
        self.monitoring = Recorder(self)
        self.field_navigation = FieldNavigation(self, self.monitoring)
        self.straight_line_navigation = StraightLineNavigation(self, self.monitoring)
        self.follow_crops_navigation = FollowCropsNavigation(self, self.monitoring)
        self.navigation_strategies = {n.name: n for n in [self.field_navigation,
                                                          self.straight_line_navigation,
                                                          self.follow_crops_navigation,
                                                          ]}
        self.weeding_implements: list[Implement] = [self.monitoring]
        match self.field_friend.implement_name:
            case 'tornado':
                self.weeding_implements.append(Tornado(self))
            case 'weed_screw':
                self.weeding_implements.append(WeedingScrew(self))
            case 'dual_mechanism':
                self.weeding_implements.append(WeedingScrew(self))
                self.weeding_implements.append(ChopAndScrew(self))
            case 'none':
                self.weeding_implements.append(WeedingScrew(self))
            case _:
                raise NotImplementedError(f'Unknown tool: {self.field_friend.implement_name}')
        # TODO reactivate other tools
        # self.coin_collecting = CoinCollecting(self)
        # self.mowing = Mowing(self, robot_width=width, shape=self.shape)
        # self.path_recorder = PathRecorder(self.path_provider, self.driver, self.steerer, self.gnss)
        tools: list[Implement] = self.weeding_implements  # + [self.coin_collecting, self.mowing]
        self.implements = {t.name: t for t in tools}
        self._current_navigation: Navigation = self.straight_line_navigation
        self._current_implement = self._current_navigation
        self.automator.default_automation = self._current_navigation.start
        self.info = Info(self)
        self.current_implement = self.monitoring
        if self.field_friend.bumper:
            self.automation_watcher.bumper_watch_active = True

        if self.is_real:
            assert isinstance(self.field_friend, FieldFriendHardware)
            if self.field_friend.battery_control:
                self.battery_watcher = BatteryWatcher(self.field_friend, self.automator)
            rosys.automation.app_controls(self.field_friend.robot_brain, self.automator)

    def restart(self) -> None:
        os.utime('main.py')

    def backup(self) -> dict:
        return {
            'navigation': self.current_navigation.name,
            'implement': self.current_implement.name,
        }

    def restore(self, data: dict[str, Any]) -> None:
        implement = self.implements.get(data.get('implement', None), None)
        if implement is not None:
            self.current_implement = implement
        navigation = self.navigation_strategies.get(data.get('navigation', None), None)
        if navigation is not None:
            self.current_navigation = navigation

    @property
    def current_implement(self) -> Implement:
        return self.current_navigation.implement

    @current_implement.setter
    def current_implement(self, implement: Implement) -> None:
        self.current_navigation.implement = implement
        self.request_backup()
        self.log.info(f'selected implement: {implement.name}')

    @property
    def current_navigation(self) -> Navigation:
        return self._current_navigation

    @current_navigation.setter
    def current_navigation(self, navigation: Navigation) -> None:
        old_navigation = self._current_navigation
        if old_navigation is not None:
            implement = self.current_implement
            navigation.implement = implement
        self._current_navigation = navigation
        self.automator.default_automation = self._current_navigation.start
        self.AUTOMATION_CHANGED.emit(navigation.name)
        self.request_backup()

    async def setup_simulated_usb_camera(self):
        self.usb_camera_provider.remove_all_cameras()
        camera = SimulatedCam.create_calibrated(id='bottom_cam',
                                                x=0.4, z=0.4,
                                                roll=np.deg2rad(360-150),
                                                pitch=np.deg2rad(0),
                                                yaw=np.deg2rad(90))
        self.usb_camera_provider.add_camera(camera)
        self.odometer.ROBOT_MOVED.register(lambda: camera.update_calibration(self.odometer.prediction))
