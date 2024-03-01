import logging
from copy import deepcopy
from typing import TYPE_CHECKING, Optional

import rosys
from rosys.geometry import Pose
from shapely.geometry import Polygon

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

        self.try_resume_active: bool = False
        self.incidence_time: float = 0.0
        self.incidence_pose: Pose = Pose()
        self.resume_delay: float = DEFAULT_RESUME_DELAY
        self.field_boundaries: list = []
        self.field_polygong: Optional[Polygon] = None

        rosys.on_repeat(self.try_resume, 0.1)
        rosys.on_repeat(self.check_field_bounds, 1.0)
        if self.field_friend.bumper:
            self.field_friend.bumper.BUMPER_TRIGGERED.register(
                lambda name: self.pause(f'the {name} bumper was triggered'))

    def pause(self, reason: str) -> None:
        if self.automator.is_running:
            self.automator.pause(because=f'{reason} (waiting {self.resume_delay:.0f}s)')
            self.try_resume_active = True
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.odometer.prediction)

    def stop(self, reason: str) -> None:
        if self.automator.is_running:
            self.automator.stop(because=f'{reason}')
            self.try_resume_active = False

    def try_resume(self) -> None:
        if self.field_friend.bumper:
            self.automator.enabled = not bool(self.field_friend.bumper.active_bumpers)

            if self.try_resume_active and self.automator.is_running:
                self.log.info('disabling auto-resume because automation is already running again')
                self.try_resume_active = False

            if self.try_resume_active and rosys.time() > self.incidence_time + self.resume_delay and not self.field_friend.bumper.active_bumpers:
                self.log.info(f'resuming automation after {self.resume_delay:.0f}s')
                self.automator.resume()
                self.resume_delay += 2
                self.try_resume_active = False

            if self.odometer.prediction.distance(self.incidence_pose) > RESET_POSE_DISTANCE:
                if self.resume_delay != DEFAULT_RESUME_DELAY:
                    self.log.info('resetting resume_delay')
                    self.resume_delay = DEFAULT_RESUME_DELAY

    def check_field_bounds(self) -> None:
        if not self.field_boundaries:
            return
        if not self.field_polygong:
            boundary_points = [(point.x, point.y) for point in self.field_boundaries]
            self.field_polygong = Polygon(boundary_points)
        position = deepcopy(self.odometer.prediction.point)
        if not self.field_polygong.contains(position):
            self.log.warning('robot is outside of field boundaries')
            self.stop('robot is outside of field boundaries')
