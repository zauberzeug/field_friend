import logging
from copy import deepcopy

import rosys
from rosys.automation import Automator
from rosys.driving import Odometer
from rosys.geometry import Pose
from rosys.hardware import Bumper

DEFAULT_RESUME_DELAY = 1.0
RESET_POSE_DISTANCE = 1.0


class AutomationWatcher:

    def __init__(self, automator: Automator, odometer: Odometer, bumper: Bumper) -> None:
        self.log = logging.getLogger('field_friend.automation_watcher')

        self.automator = automator
        self.odometer = odometer
        self.bumper = bumper

        self.is_active: bool = False
        self.incidence_time: float = 0.0
        self.incidence_pose: Pose = Pose()
        self.resume_delay: float = DEFAULT_RESUME_DELAY

        rosys.on_repeat(self.try_resume, 0.1)

        self.bumper.BUMPER_TRIGGERED.register(lambda name: self.pause(f'the {name} bumper was triggered'))

    def pause(self, reason: str) -> None:
        if self.automator.is_running:
            self.automator.pause(because=f'{reason} (waiting {self.resume_delay:.0f}s)')
            self.is_active = True
        self.incidence_time = rosys.time()
        self.incidence_pose = deepcopy(self.odometer.prediction)

    def try_resume(self) -> None:
        self.automator.enabled = not bool(self.bumper.active_bumpers)

        if self.is_active and self.automator.is_running:
            self.log.info('disabling auto-resume because automation is already running again')
            self.is_active = False

        if self.is_active and rosys.time() > self.incidence_time + self.resume_delay and not self.bumper.active_bumpers:
            self.log.info(f'resuming automation after {self.resume_delay:.0f}s')
            self.automator.resume()
            self.resume_delay += 2
            self.is_active = False

        if self.odometer.prediction.distance(self.incidence_pose) > RESET_POSE_DISTANCE:
            if self.resume_delay != DEFAULT_RESUME_DELAY:
                self.log.info('resetting resume_delay')
                self.resume_delay = DEFAULT_RESUME_DELAY
