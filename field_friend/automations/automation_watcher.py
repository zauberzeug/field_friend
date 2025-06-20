from __future__ import annotations

import logging
from copy import deepcopy
from typing import TYPE_CHECKING

import rosys
from rosys.geometry import GeoPoint, Pose
from rosys.hardware.gnss import GpsQuality
from shapely.geometry import Point as ShapelyPoint
from shapely.geometry import Polygon as ShapelyPolygon

if TYPE_CHECKING:
    from ..system import System

DEFAULT_RESUME_DELAY = 1.0
RESET_POSE_DISTANCE = 1.0


class AutomationWatcher:

    def __init__(self, system: System) -> None:
        self.log = logging.getLogger('field_friend.automation_watcher')

        self.automator = system.automator
        self.robot_locator = system.robot_locator
        self.field_friend = system.field_friend
        self.gnss = system.gnss
        self.steerer = system.steerer
        # self.path_recorder = system.path_recorder

        self.try_resume_active: bool = False
        self.incidence_time: float = 0.0
        self.incidence_pose: Pose = Pose()
        self.resume_delay: float = DEFAULT_RESUME_DELAY
        self.field_polygon: ShapelyPolygon | None = None
        self.kpi_provider = system.kpi_provider

        self.bumper_watch_active: bool = False
        self.gnss_watch_active: bool = False
        self.field_watch_active: bool = False

        self.start_time: float | None = None
        rosys.on_repeat(self._update_time, 0.1)
        rosys.on_repeat(self.try_resume, 0.1)
        rosys.on_repeat(self.check_field_bounds, 1.0)
        rosys.on_repeat(self.check_gnss, 0.1)
        if self.field_friend.bumper:
            self.field_friend.bumper.BUMPER_TRIGGERED.register(lambda name: self.pause(f'Bumper {name} was triggered'))
        self.steerer.STEERING_STARTED.register(lambda: self.pause('steering started'))
        # self.field_friend.estop.ESTOP_TRIGGERED.register(lambda: self.stop('emergency stop triggered'))

    def pause(self, reason: str) -> None:
        # TODO re-think integration of path recorder
        # dont pause automator if steering is active and path_recorder is recording
        # if reason.startswith('steering'):
        #     if self.path_recorder.state == 'recording':
        #         return
        #     else:
        #         if self.automator.is_running:
        #             self.log.info(f'pausing automation because {reason}')
        #             self.automator.pause(because=f'{reason})')
        #         return
        if reason.startswith('GNSS') and not self.gnss_watch_active:
            self.log.info(f'not pausing automation because {reason} but GNSS watch is not active')
            return
        if reason.startswith('Bumper') and not self.bumper_watch_active:
            self.log.info(f'not pausing automation because {reason} but bumper watch is not active')
            return
        if self.automator.is_running:
            self.automator.pause(because=f'{reason} (waiting {self.resume_delay:.0f}s)')
            self.try_resume_active = True
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.robot_locator.pose)

    def stop(self, reason: str) -> None:
        if self.automator.is_running:
            self.automator.stop(because=f'{reason}')
            self.try_resume_active = False
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.robot_locator.pose)

    def is_gnss_ready(self) -> bool:
        assert self.gnss is not None
        return self.gnss.is_connected \
            and self.gnss.last_measurement is not None \
            and rosys.time() - self.gnss.last_measurement.time < 2.0 \
            and self.gnss.last_measurement.gps_quality == GpsQuality.RTK_FIXED

    def try_resume(self) -> None:
        # Set conditions to True by default, which means they don't block the process if the watch is not active
        bumper_condition = self.field_friend.bumper is None or not self.bumper_watch_active or not self.field_friend.bumper.active_bumpers
        gnss_condition = self.is_gnss_ready() if self.gnss_watch_active else True

        # Enable automator only if all relevant conditions are True
        self.automator.enabled = bumper_condition and gnss_condition

        if self.try_resume_active and self.automator.is_running:
            self.log.info('disabling auto-resume because automation is already running again')
            self.try_resume_active = False

        if self.try_resume_active and rosys.time() > self.incidence_time + self.resume_delay:
            if not bumper_condition or not gnss_condition:
                self.log.info(f'waiting for conditions to be met: bumper={bumper_condition}, gnss={gnss_condition}')
                self.resume_delay += 2
                return
            self.log.info(f'resuming automation after {self.resume_delay:.0f}s')
            self.automator.resume()
            self.try_resume_active = False

        if self.robot_locator.pose.distance(self.incidence_pose) > RESET_POSE_DISTANCE:
            if self.resume_delay != DEFAULT_RESUME_DELAY:
                self.log.debug('resetting resume_delay')
                self.resume_delay = DEFAULT_RESUME_DELAY

    def start_field_watch(self, field_boundaries: list[GeoPoint]) -> None:
        self.field_polygon = ShapelyPolygon([point.to_local().tuple for point in field_boundaries])
        self.field_watch_active = True

    def stop_field_watch(self) -> None:
        self.field_watch_active = False
        self.field_polygon = None

    def check_gnss(self) -> None:
        if not self.gnss_watch_active:
            return
        if self.is_gnss_ready():
            return
        # TODO: use gps_quality here? or better robot locator uncertainty?
        self.pause('GNSS failed')

    def check_field_bounds(self) -> None:
        if not self.field_watch_active or not self.field_polygon:
            return
        position = ShapelyPoint(self.robot_locator.pose.x, self.robot_locator.pose.y)
        if not self.field_polygon.contains(position):
            self.log.debug(f'robot at {position} is outside of field boundaries {self.field_polygon}')
            if self.automator.is_running:
                self.stop('robot is outside of field boundaries')
                self.field_watch_active = False

    def _update_time(self):
        """Update KPIs for time"""
        if not self.automator.is_running:
            self.start_time = None
            return
        if self.start_time is None:
            self.start_time = rosys.time()
        passed_time = rosys.time() - self.start_time
        if passed_time > 1:
            self.kpi_provider.increment_all_time_kpi('time', passed_time)
            self.start_time = rosys.time()
