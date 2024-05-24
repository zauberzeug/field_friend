import logging
import os
from typing import Any

import numpy as np
import rosys

from field_friend.hardware import FieldFriendHardware, FieldFriendSimulation
from field_friend.navigation.gnss_hardware import GnssHardware
from field_friend.navigation.gnss_simulation import GnssSimulation
from field_friend.vision import (
    CalibratableUsbCameraProvider,
    CameraConfigurator,
    SimulatedCam,
    SimulatedCamProvider,
)

from .automations import (
    AutomationWatcher,
    BatteryWatcher,
    CoinCollecting,
    FieldProvider,
    KpiProvider,
    Mowing,
    PathProvider,
    PathRecorder,
    PlantLocator,
    PlantProvider,
    Puncher,
)
from .automations.navigation import FieldNavigation, StraightLineNavigation
from .automations.tool import ChopAndScrew, Recorder, Screw, Tool, Tornado
from .interface.components.info import Info
from .kpi_generator import generate_kpis


class System(rosys.persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.AUTOMATION_CHANGED = rosys.event.Event()

        self.field_friend: FieldFriendHardware | FieldFriendSimulation
        self.usb_camera_provider: CalibratableUsbCameraProvider | SimulatedCamProvider
        self.detector: rosys.vision.DetectorHardware | rosys.vision.DetectorSimulation
        if self.is_real:
            self.field_friend = FieldFriendHardware()
            self.usb_camera_provider = CalibratableUsbCameraProvider()
            self.mjpeg_camera_provider = rosys.vision.MjpegCameraProvider(username='root', password='zauberzg!')
            self.detector = rosys.vision.DetectorHardware(port=8004)
            self.monitoring_detector = rosys.vision.DetectorHardware(port=8005)
            self.camera_configurator = CameraConfigurator(self.usb_camera_provider)
        else:
            version = 'rb34'  # insert here your field friend version to be simulated
            self.field_friend = FieldFriendSimulation(robot_id=version)
            self.usb_camera_provider = SimulatedCamProvider()
            # NOTE we run this in rosys.startup to enforce setup AFTER the persistence is loaded
            rosys.on_startup(self.setup_simulated_usb_camera)
            self.detector = rosys.vision.DetectorSimulation(self.usb_camera_provider)
            self.camera_configurator = CameraConfigurator(self.usb_camera_provider, robot_id=version)
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
        self.crop_category_names = ['coin_with_hole', 'crop', 'sugar_beet', 'onion', 'garlic', ]
        self.plant_locator = PlantLocator(self.usb_camera_provider,
                                          self.detector,
                                          self.plant_provider,
                                          self.odometer,
                                          )
        self.plant_locator.weed_category_names = self.big_weed_category_names + self.small_weed_category_names
        self.plant_locator.crop_category_names = self.crop_category_names
        if self.field_friend.tool == 'tornado':
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
        self.monitoring = Recorder(self)
        self.field_navigation = FieldNavigation(self.driver,
                                                self.odometer,
                                                self.monitoring,
                                                self.shape,
                                                self.gnss,
                                                self.field_friend.bms)
        self.straight_line_navigation = StraightLineNavigation(self.driver, self.odometer, self.monitoring)
        self.weeding_tools: list[Tool] = [self.monitoring]
        match self.field_friend.tool:
            case 'tornado':
                self.weeding_tools.append(Tornado(self))
            case 'weed_screw':
                self.weeding_tools.append(Screw(self))
            case 'dual_mechanism':
                self.weeding_tools.append(Screw(self))
                self.weeding_tools.append(ChopAndScrew(self))
            case 'none':
                self.weeding_tools.append(Screw(self))
            case _:
                raise NotImplementedError(f'Unknown tool: {self.field_friend.tool}')
        # TODO reactivate other tools
        # self.coin_collecting = CoinCollecting(self)
        # self.mowing = Mowing(self, robot_width=width, shape=self.shape)
        # self.path_recorder = PathRecorder(self.path_provider, self.driver, self.steerer, self.gnss)
        tools: list[Tool] = self.weeding_tools  # + [self.coin_collecting, self.mowing]
        self.tools = {t.name: t for t in tools}
        self.field_navigation.tool = Recorder(self)
        self.straight_line_navigation.tool = Recorder(self)
        self.automator = rosys.automation.Automator(None, on_interrupt=lambda _: self.field_friend.stop(),
                                                    default_automation=self.straight_line_navigation.start)
        self.info = Info(self)
        self.automation_watcher = AutomationWatcher(self)
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
        return {'automation': self.get_current_automation_id()}

    def restore(self, data: dict[str, Any]) -> None:
        name = data.get('automation', None)
        tool = self.tools.get(name, None)
        # TODO restore last selected tool
        # if tool is not None:
        #     self.automator.default_automation = tool.start

    def get_current_automation_id(self) -> str | None:
        return 'none'
        if self.automator.default_automation is None:
            return None
        return {v.start: k for k, v in self.tools.items()}.get(self.automator.default_automation, None)

    def setup_simulated_usb_camera(self):
        self.usb_camera_provider.remove_all_cameras()
        self.usb_camera_provider.add_camera(SimulatedCam.create_calibrated(id='bottom_cam',
                                                                           x=0.4, z=0.4,
                                                                           roll=np.deg2rad(360-150),
                                                                           pitch=np.deg2rad(0),
                                                                           yaw=np.deg2rad(90)))
