import logging
from copy import deepcopy
from typing import TYPE_CHECKING, Optional

import rosys
from rosys.geometry import Pose
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Polygon as ShapelyPolygon

if TYPE_CHECKING:
    from system import System

DEFAULT_RESUME_DELAY = 1.0
RESET_POSE_DISTANCE = 1.0


class AutomationWatcher:

    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('field_friend.automation_watcher')

        self.automator = system.automator
        self.odometer = system.odometer
        self.field_friend = system.field_friend
        self.gnss = system.gnss
        self.steerer = system.steerer
        self.path_recorder = system.path_recorder
        self.imu = system.field_friend.imu

        self.try_resume_active: bool = False
        self.incidence_time: float = 0.0
        self.incidence_pose: Pose = Pose()
        self.resume_delay: float = DEFAULT_RESUME_DELAY
        self.field_polygong: Optional[ShapelyPolygon] = None

        self.bumper_watch_active: bool = False
        self.gnss_watch_active: bool = False
        self.field_watch_active: bool = False
        self.tilt_watch_active: bool = False

        rosys.on_repeat(self.try_resume, 0.1)
        rosys.on_repeat(self.check_field_bounds, 1.0)
        rosys.on_repeat(self.check_tilting, 0.5)
        if self.field_friend.bumper:
            self.field_friend.bumper.BUMPER_TRIGGERED.register(
                lambda name: self.pause(f'Bumper {name} was triggered'))
        self.gnss.GNSS_CONNECTION_LOST.register(lambda: self.pause('GNSS connection lost'))
        self.gnss.RTK_FIX_LOST.register(lambda: self.pause('GNSS RTK fix lost'))

        self.steerer.STEERING_STARTED.register(lambda: self.pause('steering started'))
        self.field_friend.estop.ESTOP_TRIGGERED.register(lambda: self.stop('emergency stop triggered'))

    def pause(self, reason: str) -> None:
        # dont pause automator if steering is active and path_recorder is recording
        if reason.startswith('steering'):
            if self.path_recorder.state == 'recording':
                return
            else:
                self.log.info(f'pausing automation because {reason}')
                self.automator.pause(because=f'{reason})')
                return
        if reason.startswith('GNSS') and not self.gnss_watch_active:
            return
        if reason.startswith('Bumper') and not self.bumper_watch_active:
            return
        if self.automator.is_running:
            self.automator.pause(because=f'{reason} (waiting {self.resume_delay:.0f}s)')
            self.try_resume_active = True
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.odometer.prediction)

    def stop(self, reason: str) -> None:
        if self.automator.is_running:
            self.automator.stop(because=f'{reason}')
            self.try_resume_active = False
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.odometer.prediction)

    def try_resume(self) -> None:
        # Set conditions to True by default, which means they don't block the process if the watch is not active
        bumper_condition = not bool(self.field_friend.bumper.active_bumpers) if self.bumper_watch_active else True
        gnss_condition = (self.gnss.record.gps_qual == 4 or self.gnss.record.gps_qual ==
                          8) if self.gnss_watch_active else True

        # Enable automator only if all relevant conditions are True
        self.automator.enabled = bumper_condition and gnss_condition

        if self.try_resume_active and self.automator.is_running:
            self.log.info('disabling auto-resume because automation is already running again')
            self.try_resume_active = False

        if self.try_resume_active and rosys.time() > self.incidence_time + self.resume_delay:
            if not bumper_condition or not gnss_condition:
                self.log.info(f'waiting for conditions to be met: bumper={bumper_condition}, gnss={gnss_condition}')
                return
            self.log.info(f'resuming automation after {self.resume_delay:.0f}s')
            self.automator.resume()
            self.resume_delay += 2
            self.try_resume_active = False

        if self.odometer.prediction.distance(self.incidence_pose) > RESET_POSE_DISTANCE:
            if self.resume_delay != DEFAULT_RESUME_DELAY:
                self.log.info('resetting resume_delay')
                self.resume_delay = DEFAULT_RESUME_DELAY

    def start_field_watch(self, field_boundaries: list[rosys.geometry.Point]) -> None:
        self.field_polygong = ShapelyPolygon([(point.x, point.y) for point in field_boundaries])
        self.field_watch_active = True

    def stop_field_watch(self) -> None:
        self.field_watch_active = False
        self.field_polygong = None

    def check_field_bounds(self) -> None:
        if not self.field_watch_active or not self.field_polygong:
            return
        position = ShapelyPoint(self.odometer.prediction.x, self.odometer.prediction.y)
        if not self.field_polygong.contains(position):
            self.log.warning('robot is outside of field boundaries')
            if self.automator.is_running:
                self.stop('robot is outside of field boundaries')
                self.field_watch_active = False

    def start_tilt_watch(self) -> None:
        self.tilt_watch_active = True

    def stop_tilt_watch(self) -> None:
        self.tilt_watch_active = False

    def check_tilting(self) -> None:
        if not self.imu.euler or not self.tilt_watch_active:
            return
        if not self.field_friend.ROLL_LIMIT or not self.field_friend.PITCH_LIMIT:
            return
        roll, pitch, yaw = self.imu.euler
        if (roll > self.field_friend.ROLL_LIMIT):
            self.log.info(f'robot exeeds roll limit with {roll}')
            self.stop(f'exeedet roll limit with {roll}')

        if (pitch > self.field_friend.PITCH_LIMIT):
            self.log.info(f'robot exeeds pitch limit with {pitch}')
            self.stop(f'exeedet pitch limit with {pitch}')
