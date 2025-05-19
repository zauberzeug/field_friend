import logging
import os
from typing import Any

import icecream
import numpy as np
import psutil
import rosys
from rosys.driving import Odometer
from rosys.event import Event
from rosys.geometry import GeoPoint, GeoReference
from rosys.hardware.gnss import GnssHardware, GnssSimulation

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
from .automations.implements import Implement, Recorder, Tornado, WeedingScrew
from .automations.navigation import (
    CrossglideDemoNavigation,
    FieldNavigation,
    FollowCropsNavigation,
    Navigation,
    StraightLineNavigation,
)
from .config import get_config
from .hardware import AxisD1, FieldFriend, FieldFriendHardware, FieldFriendSimulation, TeltonikaRouter
from .info import Info
from .kpi_generator import generate_kpis
from .robot_locator import RobotLocator
from .vision import CalibratableUsbCameraProvider, CameraConfigurator
from .vision.zedxmini_camera import ZedxminiCameraProvider

icecream.install()


class System(rosys.persistence.Persistable):

    def __init__(self, robot_id: str, *, restore_persistence: bool = True) -> None:
        super().__init__()
        self.robot_id = robot_id
        assert self.robot_id != 'unknown'
        self.restore_persistence = restore_persistence
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.AUTOMATION_CHANGED: Event[str] = Event()
        self.config = get_config(self.robot_id)

        self.camera_provider = self.setup_camera_provider()
        self.detector: rosys.vision.DetectorHardware | rosys.vision.DetectorSimulation | None = None
        self.update_gnss_reference(reference=GeoReference(GeoPoint.from_degrees(51.983204032849706, 7.434321368936861)))
        self.gnss: GnssHardware | GnssSimulation | None = None
        self.field_friend: FieldFriend
        self._current_navigation: Navigation | None = None
        self.implements: dict[str, Implement] = {}
        self.navigation_strategies: dict[str, Navigation] = {}
        if self.is_real:
            try:
                self.field_friend = FieldFriendHardware(self.config)
                self.teltonika_router = TeltonikaRouter()
            except Exception:
                self.log.exception(f'failed to initialize FieldFriendHardware {self.robot_id}')
            assert isinstance(self.field_friend, FieldFriendHardware)
            self.gnss = self.setup_gnss()
            self.robot_locator = RobotLocator(self.field_friend.wheels, self.gnss, self.field_friend.imu).persistent(restore=self.restore_persistence)
            self.mjpeg_camera_provider = rosys.vision.MjpegCameraProvider(username='root', password='zauberzg!')
            self.detector = rosys.vision.DetectorHardware(port=8004)
            self.monitoring_detector = rosys.vision.DetectorHardware(port=8005)
        else:
            self.field_friend = FieldFriendSimulation(self.config)
            assert isinstance(self.field_friend.wheels, rosys.hardware.WheelsSimulation)
            self.gnss = self.setup_gnss(self.field_friend.wheels)
            self.robot_locator = RobotLocator(self.field_friend.wheels, self.gnss, self.field_friend.imu).persistent(restore=self.restore_persistence)
            # NOTE we run this in rosys.startup to enforce setup AFTER the persistence is loaded
            rosys.on_startup(self.setup_simulated_usb_camera)
            if self.camera_provider is not None:
                self.detector = rosys.vision.DetectorSimulation(self.camera_provider)

        if self.config.camera is not None:
            assert self.camera_provider is not None
            self.camera_configurator = CameraConfigurator(
                self.camera_provider, robot_locator=self.robot_locator, robot_id=self.robot_id, camera_config=self.config.camera)
        self.odometer = Odometer(self.field_friend.wheels)
        self.setup_driver()
        self.plant_provider = PlantProvider().persistent(restore=self.restore_persistence)
        self.kpi_provider = KpiProvider().persistent(restore=self.restore_persistence)
        if not self.is_real:
            generate_kpis(self.kpi_provider)

        def watch_robot() -> None:
            if self.field_friend.bumper:
                self.kpi_provider.increment_on_rising_edge('bumps', bool(self.field_friend.bumper.active_bumpers))
            if self.field_friend.bms:
                self.kpi_provider.increment_on_rising_edge('low_battery', self.field_friend.bms.is_below_percent(10.0))

        self.puncher: Puncher = Puncher(self.field_friend, self.driver)
        self.plant_locator: PlantLocator = PlantLocator(self).persistent(restore=self.restore_persistence)

        rosys.on_repeat(watch_robot, 1.0)

        self.path_provider: PathProvider = PathProvider().persistent(restore=self.restore_persistence)
        self.field_provider: FieldProvider = FieldProvider().persistent(restore=self.restore_persistence)
        self.setup_shape()
        self.automator: rosys.automation.Automator = rosys.automation.Automator(
            self.steerer, on_interrupt=self.field_friend.stop)
        self.automation_watcher: AutomationWatcher = AutomationWatcher(self)

        self.setup_timelapse()
        self.setup_implements()
        self.setup_navigations()
        self.info = Info(self)
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

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'navigation': self.current_navigation.name if self.current_navigation is not None else None,
            'implement': self.current_implement.name if self.current_implement is not None else None,
            'gnss_reference': GeoReference.current.degree_tuple if GeoReference.current is not None else None,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        implement = self.implements.get(data.get('implement', None), None)
        if implement is not None:
            self.current_implement = implement
        navigation = self.navigation_strategies.get(data.get('navigation', None), None)
        if navigation is not None:
            self.current_navigation = navigation

        reference_tuple = None
        if 'gnss_reference' in data:
            reference_tuple = data['gnss_reference']
        elif 'reference_lat' in data and 'reference_long' in data:
            reference_tuple = (np.deg2rad(data['reference_lat']), np.deg2rad(data['reference_long']), 0)
        if reference_tuple is not None:
            reference = GeoReference(origin=GeoPoint.from_degrees(reference_tuple[0], reference_tuple[1]),
                                     direction=np.deg2rad(reference_tuple[2] if len(reference_tuple) > 2 else 0))
            self.update_gnss_reference(reference=reference)

    @property
    def current_implement(self) -> Implement | None:
        if self.current_navigation is None:
            return None
        return self.current_navigation.implement

    @current_implement.setter
    def current_implement(self, implement: Implement) -> None:
        if self.current_navigation is None:
            self.log.error('No navigation selected')
            return
        self.current_navigation.implement = implement
        self.request_backup()
        self.log.debug(f'selected implement: {implement.name}')

    @property
    def current_navigation(self) -> Navigation | None:
        return self._current_navigation

    @current_navigation.setter
    def current_navigation(self, navigation: Navigation) -> None:
        old_navigation = self._current_navigation
        if old_navigation is not None and self.current_implement is not None:
            implement = self.current_implement
            navigation.implement = implement
        self._current_navigation = navigation
        self.automator.default_automation = self._current_navigation.start
        self.AUTOMATION_CHANGED.emit(navigation.name)
        self.request_backup()

    def setup_driver(self) -> None:
        self.odometer = Odometer(self.field_friend.wheels)
        self.steerer = rosys.driving.Steerer(self.field_friend.wheels, speed_scaling=0.25)
        self.driver = rosys.driving.Driver(self.field_friend.wheels, self.robot_locator)
        self.driver.parameters.linear_speed_limit = 0.3
        self.driver.parameters.angular_speed_limit = 0.2
        self.driver.parameters.can_drive_backwards = True
        self.driver.parameters.minimum_turning_radius = 0.01
        self.driver.parameters.hook_offset = 0.45
        self.driver.parameters.carrot_distance = 0.15
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
        self.driver.parameters.hook_bending_factor = 0.25

    def setup_shape(self) -> None:
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

    def setup_implements(self) -> None:
        persistence_key = 'field_friend.automations.implements.weeding'
        implements: list[Implement] = []
        match self.field_friend.implement_name:
            case 'tornado':
                implements.append(Recorder(self))
                implements.append(Tornado(self).persistent(key=persistence_key, restore=self.restore_persistence))
            case 'weed_screw':
                implements.append(Recorder(self))
                implements.append(WeedingScrew(self).persistent(key=persistence_key, restore=self.restore_persistence))
            case 'dual_mechanism':
                # implements.append(WeedingScrew(self))
                # implements.append(ChopAndScrew(self))
                self.log.error('Dual mechanism not implemented')
                implements.append(Recorder(self))
            case 'recorder':
                implements.append(Recorder(self))
            case None:
                implements.append(Implement())
            case _:
                raise NotImplementedError(f'Unknown implement: {self.field_friend.implement_name}')
        self.implements = {t.name: t for t in implements}

    def setup_navigations(self) -> None:
        first_implement = next(iter(self.implements.values()))
        self.straight_line_navigation = StraightLineNavigation(self, first_implement).persistent(restore=self.restore_persistence)
        self.follow_crops_navigation = FollowCropsNavigation(self, first_implement).persistent(restore=self.restore_persistence)
        self.field_navigation = FieldNavigation(self, first_implement).persistent(restore=self.restore_persistence) if self.gnss is not None else None
        self.crossglide_demo_navigation = CrossglideDemoNavigation(self, first_implement).persistent(restore=self.restore_persistence) \
            if isinstance(self.field_friend.y_axis, AxisD1) else None
        self.navigation_strategies = {n.name: n for n in [self.straight_line_navigation,
                                                          self.follow_crops_navigation,
                                                          self.field_navigation,
                                                          self.crossglide_demo_navigation,
                                                          ] if n is not None}
        self.current_navigation = self.straight_line_navigation

    def setup_camera_provider(self) -> CalibratableUsbCameraProvider | rosys.vision.SimulatedCameraProvider | ZedxminiCameraProvider | None:
        if not self.is_real:
            return rosys.vision.SimulatedCameraProvider()
        if self.config.camera is None:
            self.log.warning('Camera is not configured, no camera provider will be used')
            return None
        if self.config.camera.camera_type == 'CalibratableUsbCamera':
            return CalibratableUsbCameraProvider().persistent(restore=self.restore_persistence)
        if self.config.camera.camera_type == 'ZedxminiCamera':
            return ZedxminiCameraProvider().persistent(restore=self.restore_persistence)
        raise NotImplementedError(f'Unknown camera type: {self.config.camera.camera_type}')

    async def setup_simulated_usb_camera(self):
        if self.camera_provider is None:
            self.log.error('No camera provider configured, skipping simulated USB camera setup')
            return
        self.camera_provider.remove_all_cameras()
        camera = rosys.vision.SimulatedCalibratableCamera.create_calibrated(id='bottom_cam',
                                                                            x=0.4, z=0.4,
                                                                            roll=np.deg2rad(360-150),
                                                                            pitch=np.deg2rad(0),
                                                                            yaw=np.deg2rad(90),
                                                                            color='#cccccc',
                                                                            frame=self.robot_locator.pose_frame,
                                                                            )
        assert isinstance(self.camera_provider, rosys.vision.SimulatedCameraProvider)
        self.camera_provider.add_camera(camera)

    def setup_timelapse(self) -> None:
        self.timelapse_recorder = rosys.analysis.TimelapseRecorder()
        self.timelapse_recorder.frame_info_builder = lambda _: f'''{self.robot_id}, {self.current_navigation.name if self.current_navigation is not None else 'No Navigation'}, \
            tags: {", ".join(self.plant_locator.tags)}'''
        rosys.NEW_NOTIFICATION.register(self.timelapse_recorder.notify)
        rosys.on_startup(self.timelapse_recorder.compress_video)  # NOTE: cleanup JPEGs from before last shutdown

    def setup_gnss(self, wheels: rosys.hardware.WheelsSimulation | None = None) -> GnssHardware | GnssSimulation | None:
        if self.config.gnss is None:
            return None
        if self.is_real:
            return GnssHardware(antenna_pose=self.config.gnss.antenna_pose)
        assert isinstance(wheels, rosys.hardware.WheelsSimulation)
        return GnssSimulation(wheels=wheels, lat_std_dev=1e-10, lon_std_dev=1e-10, heading_std_dev=1e-10)

    def update_gnss_reference(self, *, reference: GeoReference | None = None) -> None:
        if reference is None:
            if self.gnss is None:
                self.log.warning('Not updating GNSS reference: GNSS not configured')
                return
            if self.gnss.last_measurement is None:
                self.log.warning('Not updating GNSS reference: No GNSS measurement received')
                return
            reference = GeoReference(origin=self.gnss.last_measurement.point,
                                     direction=self.gnss.last_measurement.heading)
        self.log.debug('Updating GNSS reference to %s', reference)
        GeoReference.update_current(reference)
        self.request_backup()

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
