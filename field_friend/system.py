import logging
import os
from typing import Any, cast

import icecream
import numpy as np
import psutil
import requests
import rosys
from rosys.version import __version__ as rosys_version

import config.config_selection as config_selector

from . import localization
from .app_controls import AppControls as app_controls
from .automations import (
    AutomationWatcher,
    BatteryWatcher,
    FieldProvider,
    KpiProvider,
    PathProvider,
    PlantLocator,
    PlantProvider,
    Puncher,
)
from .automations.implements import ExternalMower, Implement, Recorder, Tornado, WeedingScrew
from .automations.navigation import (
    CrossglideDemoNavigation,
    FieldNavigation,
    FollowCropsNavigation,
    Navigation,
    StraightLineNavigation,
)
from .hardware import FieldFriend, FieldFriendHardware, FieldFriendSimulation, TeltonikaRouter
from .info import Info
from .kpi_generator import generate_kpis
from .localization.geo_point import GeoPoint
from .localization.gnss_hardware import GnssHardware
from .localization.gnss_simulation import GnssSimulation
from .vision import CalibratableUsbCameraProvider, CameraConfigurator
from .vision.zedxmini_camera import ZedxminiCameraProvider

icecream.install()


class System(rosys.persistence.PersistentModule):

    version = 'unknown'  # This is set in main.py through the environment variable VERSION or ROBOT_ID
    robot_id = 'unknown'  # This is set in main.py through the environment variable ROBOT_ID

    def __init__(self) -> None:
        super().__init__()
        assert self.version is not None
        assert self.version != 'unknown'
        assert self.robot_id is not None
        assert self.robot_id != 'unknown'

        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.AUTOMATION_CHANGED = rosys.event.Event()

        self.camera_provider = self.setup_camera_provider()
        self.detector: rosys.vision.DetectorHardware | rosys.vision.DetectorSimulation
        self.field_friend: FieldFriend
        if self.is_real:
            try:
                self.field_friend = FieldFriendHardware()
                self.teltonika_router = TeltonikaRouter()
            except Exception:
                self.log.exception(f'failed to initialize FieldFriendHardware {self.version}')
            self.mjpeg_camera_provider = rosys.vision.MjpegCameraProvider(username='root', password='zauberzg!')
            self.detector = rosys.vision.DetectorHardware(port=8004)
            self.monitoring_detector = rosys.vision.DetectorHardware(port=8005)
            self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
            self.camera_configurator = CameraConfigurator(self.camera_provider, odometer=self.odometer)
        else:
            self.field_friend = FieldFriendSimulation(robot_id=self.version)
            # NOTE we run this in rosys.startup to enforce setup AFTER the persistence is loaded
            rosys.on_startup(self.setup_simulated_usb_camera)
            self.detector = rosys.vision.DetectorSimulation(self.camera_provider)
            self.odometer = rosys.driving.Odometer(self.field_friend.wheels)
            self.camera_configurator = CameraConfigurator(
                self.camera_provider, odometer=self.odometer, robot_id=self.version)
        self.plant_provider = PlantProvider()
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.25)
        self.gnss: GnssHardware | GnssSimulation
        if self.is_real:
            assert isinstance(self.field_friend, FieldFriendHardware)
            self.gnss = GnssHardware(self.odometer, self.field_friend.ANTENNA_OFFSET)
        else:
            assert isinstance(self.field_friend.wheels, rosys.hardware.WheelsSimulation)
            self.gnss = GnssSimulation(self.odometer, self.field_friend.wheels)
        self.gnss.ROBOT_POSE_LOCATED.register(self.odometer.handle_detection)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.odometer)
        self.driver.parameters.linear_speed_limit = 0.3
        self.driver.parameters.angular_speed_limit = 0.2
        self.driver.parameters.can_drive_backwards = True
        self.driver.parameters.minimum_turning_radius = 0.01
        self.driver.parameters.hook_offset = 0.45
        self.driver.parameters.carrot_distance = 0.15
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
        self.driver.parameters.hook_bending_factor = 0.25

        self.kpi_provider = KpiProvider()
        if not self.is_real:
            generate_kpis(self.kpi_provider)

        def watch_robot() -> None:
            if self.field_friend.bumper:
                self.kpi_provider.increment_on_rising_edge('bumps', bool(self.field_friend.bumper.active_bumpers))
            if self.field_friend.bms:
                self.kpi_provider.increment_on_rising_edge('low_battery', self.field_friend.bms.is_below_percent(10.0))

        self.puncher = Puncher(self.field_friend, self.driver)
        self.plant_locator = PlantLocator(self)

        rosys.on_repeat(watch_robot, 1.0)
        if self.is_real and os.environ.get('SWARM_URL'):
            rosys.on_repeat(self.send_status_to_swarm, 60)
        else:
            rosys.notify('Swarm URL is not set', type='negative')

        self.path_provider = PathProvider()
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
        self.automator = rosys.automation.Automator(self.steerer, on_interrupt=self.field_friend.stop)
        self.automation_watcher = AutomationWatcher(self)
        self.monitoring = Recorder(self)
        self.timelapse_recorder = rosys.analysis.TimelapseRecorder()
        self.timelapse_recorder.frame_info_builder = lambda _: f'''{self.version}, {self.current_navigation.name}, \
            tags: {", ".join(self.plant_locator.tags)}'''
        rosys.NEW_NOTIFICATION.register(self.timelapse_recorder.notify)
        rosys.on_startup(self.timelapse_recorder.compress_video)  # NOTE: cleanup JPEGs from before last shutdown
        self.straight_line_navigation = StraightLineNavigation(self, self.monitoring)
        self.follow_crops_navigation = FollowCropsNavigation(self, self.monitoring)
        self.field_navigation = FieldNavigation(self, self.monitoring)

        self.crossglide_demo_navigation = CrossglideDemoNavigation(self, self.monitoring)
        self.navigation_strategies: dict[str, Navigation] = {n.name: n for n in [self.straight_line_navigation,
                                                                                 self.follow_crops_navigation,
                                                                                 self.field_navigation,
                                                                                 self.crossglide_demo_navigation,
                                                                                 ]}
        implements: list[Implement] = [self.monitoring]
        match self.field_friend.implement_name:
            case 'tornado':
                implements.append(Tornado(self))
            case 'weed_screw':
                implements.append(WeedingScrew(self))
            case 'dual_mechanism':
                # implements.append(WeedingScrew(self))
                # implements.append(ChopAndScrew(self))
                self.log.error('Dual mechanism not implemented')
            case 'none':
                implements.append(WeedingScrew(self))
            case 'mower':
                # TODO: mower has neither flashlight nor camera, so monitoring is not possible
                implements = [ExternalMower(self)]
            case _:
                raise NotImplementedError(f'Unknown tool: {self.field_friend.implement_name}')
        self.implements: dict[str, Implement] = {t.name: t for t in implements}
        self._current_navigation: Navigation = self.straight_line_navigation
        self._current_implement = self._current_navigation
        self.automator.default_automation = self._current_navigation.start
        self.info = Info(self)
        self.current_implement = self.monitoring
        if self.field_friend.bumper:
            self.automation_watcher.bumper_watch_active = True
        else:
            self.log.warning('Bumper is not available, does robot have bumpers?')

        if self.is_real:
            assert isinstance(self.field_friend, FieldFriendHardware)
            if self.field_friend.battery_control:
                self.battery_watcher = BatteryWatcher(self.field_friend, self.automator)
            app_controls(self.field_friend.robot_brain, self.automator, self.field_friend)
            rosys.on_repeat(self.log_status, 60 * 5)

    def restart(self) -> None:
        os.utime('main.py')

    def backup(self) -> dict:
        return {
            'navigation': self.current_navigation.name,
            'implement': self.current_implement.name,
            'reference_lat': localization.reference.lat,
            'reference_long': localization.reference.long,
        }

    def restore(self, data: dict[str, Any]) -> None:
        implement = self.implements.get(data.get('implement', None), None)
        if implement is not None:
            self.current_implement = implement
        navigation = self.navigation_strategies.get(data.get('navigation', None), None)
        if navigation is not None:
            self.current_navigation = navigation
        lat = data.get('reference_lat', 0)
        long = data.get('reference_long', 0)
        localization.reference = GeoPoint(lat=lat, long=long)

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

    def setup_camera_provider(self) -> CalibratableUsbCameraProvider | rosys.vision.SimulatedCameraProvider | ZedxminiCameraProvider:
        if not self.is_real:
            return rosys.vision.SimulatedCameraProvider()
        camera_config = config_selector.import_config(module='camera')
        camera_type = camera_config.get('type', 'CalibratableUsbCamera')
        if camera_type == 'CalibratableUsbCamera':
            return CalibratableUsbCameraProvider()
        if camera_type == 'ZedxminiCamera':
            return ZedxminiCameraProvider()
        raise NotImplementedError(f'Unknown camera type: {camera_type}')

    async def setup_simulated_usb_camera(self):
        self.camera_provider.remove_all_cameras()
        camera = rosys.vision.SimulatedCalibratableCamera.create_calibrated(id='bottom_cam',
                                                                            x=0.4, z=0.4,
                                                                            roll=np.deg2rad(360-150),
                                                                            pitch=np.deg2rad(0),
                                                                            yaw=np.deg2rad(90),
                                                                            color='#cccccc',
                                                                            frame=self.odometer.prediction_frame,
                                                                            )
        assert isinstance(self.camera_provider, rosys.vision.SimulatedCameraProvider)
        self.camera_provider.add_camera(camera)

    def get_jetson_cpu_temperature(self):
        with open('/sys/devices/virtual/thermal/thermal_zone0/temp') as f:
            temp = f.read().strip()
        return float(temp) / 1000.0  # Convert from milli °C to °C

    def log_status(self):
        msg = f'cpu: {psutil.cpu_percent():.0f}%  '
        msg += f'mem: {psutil.virtual_memory().percent:.0f}% '
        msg += f'temp: {self.get_jetson_cpu_temperature():.1f}°C '
        msg += f'battery: {self.field_friend.bms.state.short_string}'
        self.log.info(msg)

        bms_logger = logging.getLogger('field_friend.bms')
        bms_logger.info(f'Battery: {self.field_friend.bms.state.short_string}')

    async def send_status_to_swarm(self) -> None:
        swarm_url = os.environ.get('SWARM_URL')
        try:
            status = 'emergency stop' if len(self.field_friend.estop.pressed_estops) > 0 or self.field_friend.estop.is_soft_estop_active else \
                'bumper active' if self.field_friend.bumper and self.field_friend.bumper.active_bumpers else \
                'working' if self.automator.automation is not None and self.automator.automation.is_running else \
                'idle'
            if self.is_real:
                lizard_firmware = cast(FieldFriendHardware, self.field_friend).robot_brain.lizard_firmware
                await lizard_firmware.read_core_version()
                await lizard_firmware.read_p0_version()
                core_version = lizard_firmware.core_version
                p0_version = lizard_firmware.p0_version
            else:
                core_version = 'simulation'
                p0_version = 'simulation'
            if self.current_navigation is not None and self.current_navigation is FieldNavigation:
                field = self.field_navigation.field if self.field_navigation.field is not None else None
                row = self.field_navigation.current_row if self.field_navigation.current_row is not None else None
            else:
                field = None
                row = None
            position = self.gnss.current.location if self.gnss.current is not None else None
            data = {
                'version': self.version,
                'battery': self.field_friend.bms.state.percentage,
                'battery_charging': self.field_friend.bms.state.is_charging,
                'status': status,
                'position': {'lat': position.lat, 'long': position.long} if position is not None else None,
                # TODO: update the gnss quality with kalman filter
                'gnss_quality': self.gnss.current.gps_qual if self.gnss.current is not None else None,
                'implement': self.field_friend.implement_name,
                'navigation': self.current_navigation.name if self.current_navigation is not None else None,
                'field': field,
                'row': row,
                'rosys_version': rosys_version,
                'core_lizard_version': core_version,
                'p0_lizard_version': p0_version,
            }
            endpoint = f'{swarm_url}/api/robot/{self.robot_id.lower()}'
            passphrase = os.environ.get('SWARM_PASSPHRASE')
            headers = {'passphrase': passphrase} if passphrase else {}
            response = requests.post(endpoint, json=data, headers=headers, timeout=5)
            if response.status_code != 200:
                rosys.notify(f'Response code {response.status_code}.', type='negative')
        except Exception as e:
            rosys.notify(f'Error sending status: {e!s}', type='negative')
