import gc
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
from .automations import AutomationWatcher, FieldProvider, KpiProvider, PlantLocator, PlantProvider, Puncher
from .automations.implements import Implement, Recorder, Tornado, WeedingScrew, WeedingSprayer
from .automations.navigation import FieldNavigation, ImplementDemoNavigation, StraightLineNavigation, WaypointNavigation
from .capture import Capture
from .config import get_config
from .hardware import Axis, FieldFriend, FieldFriendHardware, FieldFriendSimulation, TeltonikaRouter
from .info import Info
from .robot_locator import RobotLocator
from .vision import CalibratableUsbCameraProvider, CameraConfigurator, DetectorHardware
from .vision.zedxmini_camera import ZedxminiCameraProvider

icecream.install()


class System(rosys.persistence.Persistable):

    def __init__(self, robot_id: str, *, use_acceleration: bool = False) -> None:
        super().__init__()
        self.robot_id = robot_id
        assert self.robot_id != 'unknown'
        self.config = get_config(self.robot_id)
        rosys.hardware.SerialCommunication.search_paths.insert(0, '/dev/ttyTHS0')
        self.log = logging.getLogger('field_friend.system')
        self.is_real = rosys.hardware.SerialCommunication.is_possible()
        self.AUTOMATION_CHANGED: Event[str] = Event()
        self.GNSS_REFERENCE_CHANGED: Event[[]] = Event()

        self.camera_provider = self.setup_camera_provider()
        self.detector: DetectorHardware | rosys.vision.DetectorSimulation | None = None
        self.circle_sight_detector: DetectorHardware | rosys.vision.DetectorSimulation | None = None
        self.update_gnss_reference(reference=GeoReference(GeoPoint.from_degrees(51.983204032849706, 7.434321368936861)))
        self.gnss: GnssHardware | GnssSimulation | None = None
        self.field_friend: FieldFriend
        self._current_navigation: WaypointNavigation | None = None
        self.implements: dict[str, Implement] = {}
        self.navigation_strategies: dict[str, WaypointNavigation] = {}
        self.mjpeg_camera_provider: rosys.vision.MjpegCameraProvider | None = None
        if self.is_real:
            try:
                self.field_friend = FieldFriendHardware(self.config)
                self.teltonika_router = TeltonikaRouter()
            except Exception:
                self.log.exception(f'failed to initialize FieldFriendHardware {self.robot_id}')
            assert isinstance(self.field_friend, FieldFriendHardware)
            self.gnss = self.setup_gnss()
            self.robot_locator = RobotLocator(self.field_friend.wheels,
                                              gnss=self.gnss,
                                              imu=self.field_friend.imu,
                                              gnss_config=self.config.gnss).persistent()
            self.mjpeg_camera_provider = rosys.vision.MjpegCameraProvider(username='root', password='zauberzg!')
            self.detector = DetectorHardware(self.field_friend.bms, port=8004)
            self.circle_sight_detector = DetectorHardware(self.field_friend.bms, port=8005)
        else:
            self.field_friend = FieldFriendSimulation(self.config, use_acceleration=use_acceleration)
            assert isinstance(self.field_friend.wheels, rosys.hardware.WheelsSimulation)
            self.gnss = self.setup_gnss(self.field_friend.wheels)
            self.robot_locator = RobotLocator(self.field_friend.wheels,
                                              gnss=self.gnss,
                                              imu=self.field_friend.imu,
                                              gnss_config=self.config.gnss).persistent()
            # NOTE we run this in rosys.startup to enforce setup AFTER the persistence is loaded
            rosys.on_startup(self.setup_simulated_usb_camera)
            if self.camera_provider is not None:
                self.detector = rosys.vision.DetectorSimulation(self.camera_provider)
        self.GNSS_REFERENCE_CHANGED.register(self.robot_locator.reset)
        self.capture = Capture(self)
        if self.config.camera is not None:
            assert self.camera_provider is not None
            self.camera_configurator = CameraConfigurator(self.camera_provider,
                                                          robot_locator=self.robot_locator,
                                                          robot_id=self.robot_id,
                                                          camera_config=self.config.camera)
        self.odometer = Odometer(self.field_friend.wheels)
        self.setup_driver()
        self.automator: rosys.automation.Automator = rosys.automation.Automator(
            self.steerer,
            on_interrupt=self.field_friend.stop,
            notify=False,
        )
        self.plant_provider = PlantProvider().persistent()
        self.plant_locator: PlantLocator = PlantLocator(self).persistent()
        self.puncher: Puncher = Puncher(self.field_friend, self.driver)
        self.field_provider: FieldProvider = FieldProvider().persistent()
        self.field_provider.FIELD_SELECTED.register(self.update_gnss_reference_from_field)
        self.automation_watcher: AutomationWatcher = AutomationWatcher(self)

        self.setup_timelapse()
        self.setup_kpi()
        self.setup_implements()
        self.setup_navigations()
        self.info = Info(self)
        if self.field_friend.bumper:
            self.automation_watcher.bumper_watch_active = True
        else:
            self.log.warning('Bumper is not available, does robot have bumpers?')

        if self.is_real:
            assert isinstance(self.field_friend, FieldFriendHardware)
            app_controls(self.field_friend.robot_brain, self.automator, self.field_friend, capture=self.capture)
            rosys.on_repeat(self.log_status, 60 * 5)
        rosys.on_repeat(self._garbage_collection, 60*5)
        rosys.config.garbage_collection_mbyte_limit = 0

    def restart(self) -> None:
        os.utime('main.py')

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'navigation': self.current_navigation.name if self.current_navigation is not None else None,
            'implement': self.current_implement.name if self.current_implement is not None else None,
            'gnss_reference': GeoReference.current.degree_tuple if GeoReference.current is not None else None,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        persistent_implement = data.get('implement', None)
        if persistent_implement is not None:
            implement = self.implements.get(persistent_implement, None)
            if implement is not None:
                self.current_implement = implement

        persistent_navigation = data.get('navigation', None)
        if persistent_navigation is not None:
            navigation = self.navigation_strategies.get(persistent_navigation, None)
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
    def current_navigation(self) -> WaypointNavigation | None:
        return self._current_navigation

    @current_navigation.setter
    def current_navigation(self, navigation: WaypointNavigation) -> None:
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
        self.driver.parameters.angular_speed_limit = 0.3
        self.driver.parameters.can_drive_backwards = False
        self.driver.parameters.minimum_turning_radius = 0.01
        self.driver.parameters.hook_offset = 0.20
        self.driver.parameters.carrot_distance = 0.15
        self.driver.parameters.carrot_offset = self.driver.parameters.hook_offset + self.driver.parameters.carrot_distance
        self.driver.parameters.hook_bending_factor = 0.25
        self.driver.parameters.minimum_drive_distance = 0.005
        self.driver.parameters.throttle_at_end_distance = 0.2
        self.driver.parameters.throttle_at_end_min_speed = 0.08

    def setup_implements(self) -> None:
        persistence_key = 'field_friend.automations.implements.weeding'
        implements: list[Implement] = []
        match self.field_friend.implement_name:
            case 'tornado':
                implements.append(Recorder(self))
                implements.append(Tornado(self).persistent(key=persistence_key))
            case 'weed_screw':
                implements.append(Recorder(self))
                implements.append(WeedingScrew(self).persistent(key=persistence_key))
            case 'dual_mechanism':
                # implements.append(WeedingScrew(self))
                # implements.append(ChopAndScrew(self))
                self.log.error('Dual mechanism not implemented')
                implements.append(Recorder(self))
            case 'sprayer':
                implements.append(Recorder(self))
                implements.append(WeedingSprayer(self))
            case 'recorder':
                implements.append(Recorder(self))
            case None:
                implements.append(Implement())
            case _:
                raise NotImplementedError(f'Unknown implement: {self.field_friend.implement_name}')
        self.implements = {t.name: t for t in implements}

    def setup_navigations(self) -> None:
        first_implement = next(iter(self.implements.values()))
        self.straight_line_navigation = StraightLineNavigation(self, first_implement).persistent()
        self.field_navigation = FieldNavigation(self, first_implement).persistent() if self.gnss is not None else None
        self.implement_demo_navigation = ImplementDemoNavigation(self, first_implement).persistent() \
            if isinstance(self.field_friend.y_axis, Axis) else None
        self.navigation_strategies = {n.name: n for n in [self.straight_line_navigation,
                                                          self.field_navigation,
                                                          self.implement_demo_navigation,
                                                          ] if n is not None}
        self.current_navigation = self.straight_line_navigation

    def setup_camera_provider(self) -> CalibratableUsbCameraProvider | rosys.vision.SimulatedCameraProvider | ZedxminiCameraProvider | None:
        if not self.is_real:
            return rosys.vision.SimulatedCameraProvider()
        if self.config.camera is None:
            self.log.warning('Camera is not configured, no camera provider will be used')
            return None
        if self.config.camera.camera_type == 'CalibratableUsbCamera':
            return CalibratableUsbCameraProvider().persistent()
        if self.config.camera.camera_type == 'ZedxminiCamera':
            return ZedxminiCameraProvider().persistent()
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
                                                                            frame=self.robot_locator.pose_frame)
        assert isinstance(self.camera_provider, rosys.vision.SimulatedCameraProvider)
        self.camera_provider.add_camera(camera)

    def setup_timelapse(self) -> None:
        self.timelapse_recorder = rosys.analysis.TimelapseRecorder()
        self.timelapse_recorder.frame_info_builder = lambda _: f'''{self.robot_id}, {self.current_navigation.name if self.current_navigation is not None else 'No Navigation'}, \
            tags: {", ".join(self.plant_locator.tags)}'''
        rosys.NEW_NOTIFICATION.register(self.timelapse_recorder.notify)
        rosys.on_startup(self.timelapse_recorder.compress_video)  # NOTE: cleanup JPEGs from before last shutdown

    def setup_kpi(self) -> None:
        last_update = rosys.time()
        last_position = self.robot_locator.pose
        self.kpi_provider = KpiProvider().persistent()
        if not self.is_real:
            self.kpi_provider.simulate_kpis()

        if self.automator:
            self.automator.AUTOMATION_PAUSED \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('automation_paused', 1))
            self.automator.AUTOMATION_STOPPED \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('automation_stopped', 1))
            self.automator.AUTOMATION_FAILED \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('automation_failed', 1))
            self.automator.AUTOMATION_COMPLETED \
                .register(lambda: self.kpi_provider.increment_all_time_kpi('automation_completed', 1))

        if self.plant_provider:
            self.plant_provider.ADDED_NEW_WEED \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('weeds_detected', 1))
            self.plant_provider.ADDED_NEW_CROP \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('crops_detected', 1))
        if self.puncher:
            self.puncher.PUNCHED.register(lambda: self.kpi_provider.increment_all_time_kpi('punches', 1))
        if self.field_friend.bumper:
            self.field_friend.bumper.BUMPER_TRIGGERED \
                .register(lambda _: self.kpi_provider.increment_all_time_kpi('bumps', 1))
        if self.field_friend.estop:
            self.field_friend.estop.ESTOP_TRIGGERED \
                .register(lambda: self.kpi_provider.increment_all_time_kpi('e_stop_triggered', 1))

        if self.automator:
            def gnss_failed(reason: str) -> None:
                if 'GNSS' not in reason:
                    return
                self.kpi_provider.increment_all_time_kpi('gnss_failed', 1)
            self.automator.AUTOMATION_PAUSED.register(gnss_failed)

        def watch_robot() -> None:
            nonlocal last_update, last_position
            current_time = rosys.time()
            time_since_last_update = current_time - last_update
            if self.field_friend.bms:
                self.kpi_provider.increment_on_rising_edge('low_battery', self.field_friend.bms.is_below_percent(10.0))

            if self.automator.is_running and not self.field_friend.bms.state.is_charging:
                self.kpi_provider.increment_all_time_kpi('time_working', time_since_last_update)
            if self.field_friend.bms.state.is_charging:
                self.kpi_provider.increment_all_time_kpi('time_charging', time_since_last_update)

            distance = self.robot_locator.pose.distance(last_position)
            if 0.0001 < distance < 1.0:  # NOTE: only use realistic movements, ignore i.e. gnss reference updates
                self.kpi_provider.increment_all_time_kpi('distance', distance)
            last_position = self.robot_locator.pose
            last_update = current_time

        rosys.on_repeat(watch_robot, 1.0)

    def setup_gnss(self, wheels: rosys.hardware.WheelsSimulation | None = None) -> GnssHardware | GnssSimulation | None:
        if self.config.gnss is None:
            return None
        if self.is_real:
            gnss_hardware = GnssHardware(antenna_pose=self.config.gnss.pose)
            gnss_hardware.MAX_TIMESTAMP_DIFF = 0.25
            return gnss_hardware
        assert isinstance(wheels, rosys.hardware.WheelsSimulation)
        if rosys.is_test:
            # NOTE: quick fix for https://github.com/zauberzeug/field_friend/issues/348
            return GnssSimulation(wheels=wheels, lat_std_dev=1e-10, lon_std_dev=1e-10, heading_std_dev=1e-10)
        return GnssSimulation(wheels=wheels, lat_std_dev=1e-5, lon_std_dev=1e-5, heading_std_dev=1e-5)

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
        self.GNSS_REFERENCE_CHANGED.emit()
        self.request_backup()

    def update_gnss_reference_from_field(self) -> None:
        if self.field_provider.selected_field is None:
            return
        self.update_gnss_reference(reference=self.field_provider.selected_field.geo_reference)

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

    def _garbage_collection(self):
        if self.automator.is_running:
            return
        gc.collect()
