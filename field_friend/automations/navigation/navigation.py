from __future__ import annotations

import abc
import gc
import logging
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
import rosys.geometry
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import GeoReference, Point, Pose
from rosys.hardware import Gnss

from ..implements.implement import Implement

if TYPE_CHECKING:
    from ...system import System


class WorkflowException(Exception):
    pass


class Navigation(rosys.persistence.Persistable):
    LINEAR_SPEED_LIMIT: float = 0.13
    LINEAR_SPEED_MINIMUM: float = 0.01

    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.system = system
        self.driver = system.driver
        self.robot_locator = system.robot_locator
        self.gnss = system.gnss
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.implement = implement
        self.detector = system.detector
        self.name = 'Unknown'
        self.start_position = self.robot_locator.pose.point
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT

    @property
    @abc.abstractmethod
    def target_heading(self) -> float:
        """The heading to the target point"""
        raise NotImplementedError

    @track
    async def start(self) -> None:
        try:
            if not is_reference_valid(self.gnss):
                rosys.notify('GNSS not available or reference too far away', 'warning')
                await rosys.sleep(3)
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            if not await self.implement.prepare():
                self.log.error('Tool-Preparation failed')
                return
            rosys.notify(f'Activating {self.implement.name}...')
            await self.implement.activate()
            self.start_position = self.robot_locator.pose.point
            if isinstance(self.driver.wheels, rosys.hardware.WheelsSimulation) and not rosys.is_test:
                self.create_simulation()
            self.log.info('Navigation started')

            async def get_nearest_target() -> Point:
                while True:
                    move_target = await self.implement.get_move_target()
                    if move_target and not self.implement.is_blocked:
                        return move_target
                    await rosys.sleep(0.1)

            while not self._should_finish():
                await rosys.automation.parallelize(
                    self._drive(),
                    get_nearest_target(),
                    return_when_first_completed=True,
                )
                while True:
                    move_target = await self.implement.get_move_target()
                    if not move_target:
                        self.log.warning('Stopped to weed, because no move target found')
                        break
                    move_pose = Pose(x=move_target.x, y=move_target.y, yaw=self.target_heading)
                    # TODO: using WORK_Y doesnt seem to work, we should check that
                    move_pose = move_pose.transform_pose(Pose(x=-self.system.field_friend.WORK_X, y=0, yaw=0))
                    await self.drive_towards_target(move_pose)
                    await self.implement.start_workflow()
                    await self.implement.stop_workflow()
                    await rosys.sleep(0.1)
                await rosys.sleep(0.1)
        except WorkflowException as e:
            rosys.notify(f'Navigation failed: {e}', 'negative')
        finally:
            await self.implement.finish()
            await self.finish()
            await self.implement.deactivate()
            await self.driver.wheels.stop()

    @track
    async def prepare(self) -> bool:
        """Prepares the navigation for the start of the automation

        Returns true if all preparations were successful, otherwise false."""
        self.plant_provider.clear()
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.detector.simulated_objects = []
        self.log.info('clearing plant provider')
        return True

    @track
    async def finish(self) -> None:
        """Executed after the navigation is done"""
        self.log.info('Navigation finished')
        gc.collect()

    @abc.abstractmethod
    async def _drive(self) -> None:
        """Drive the robot to the next waypoint of the navigation"""

    @track
    async def drive_towards_target(self,
                                   target: Point | Pose, *,
                                   target_heading: float | None = None,
                                   max_turn_angle: float = np.deg2rad(1.0),
                                   minimum_distance: float = 0.005) -> None:
        """Drives the robot towards a target point, but keeps the robot on a set heading with a limited turn angle.

        :param target: The target point to drive towards
        :param target_heading: The target heading to drive towards, if ``None``, the target heading of the current navigation is used
        :param max_turn_angle: The maximum turn angle in radians
        :param minimum_distance: The minimum distance to the target point in meters
        """
        if isinstance(target, Pose):
            target = target.point
        if target_heading is None:
            target_heading = self.target_heading
        if max_turn_angle != 0:
            angle_diff = rosys.helpers.eliminate_2pi(self.robot_locator.pose.direction(target) - target_heading)
            target_heading = target_heading + np.clip(angle_diff, -max_turn_angle, max_turn_angle)

        assert target_heading is not None
        line_end = self.robot_locator.pose.point.polar(self.robot_locator.pose.distance(target), target_heading)
        line_segment = rosys.geometry.LineSegment(point1=self.robot_locator.pose.point,
                                                  point2=line_end)
        adjusted_target = line_segment.line.foot_point(target)
        if self.robot_locator.pose.distance(adjusted_target) < minimum_distance:
            self.log.warning('Target too close, skipping')
            return
        relative_target = self.robot_locator.pose.relative_point(adjusted_target)
        if relative_target.x <= 0:
            self.log.warning('%s is behind the robot at %s, skipping', adjusted_target, self.robot_locator.pose)
            return
        self.log.debug('Driving towards %s with adjusted %s from %s', target, adjusted_target, self.robot_locator.pose)
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit):
            await self.driver.drive_to(adjusted_target)

    @track
    async def turn_to_yaw(self, target_yaw: float, *, angle_threshold: float = np.deg2rad(1.0)) -> None:
        """Turns the robot on the spot to a target heading.

        :param target_yaw: The target heading in radians
        :param angle_threshold: The accepted threshold around the target heading in radians, default is 1 degree
        """
        # TODO: growing error because of the threshold
        while True:
            angle = rosys.helpers.eliminate_2pi(target_yaw - self.robot_locator.pose.yaw)
            if abs(angle) < angle_threshold:
                break
            sign = 1 if angle > 0 else -1
            angular = 0.5 / self.driver.parameters.minimum_turning_radius * sign
            await self.driver.wheels.drive(*self.driver._throttle(0.0, angular))  # pylint: disable=protected-access
            await rosys.sleep(0.1)
        await self.driver.wheels.stop()

    @abc.abstractmethod
    def _should_finish(self) -> bool:
        """Returns True if the navigation should stop and be finished"""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'linear_speed_limit': self.linear_speed_limit,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)

    def create_simulation(self) -> None:
        pass

    def settings_ui(self) -> None:
        ui.number('Linear Speed',
                  step=0.01,
                  min=self.driver.parameters.throttle_at_end_min_speed,
                  max=self.driver.parameters.linear_speed_limit,
                  format='%.2f',
                  suffix='m/s',
                  on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'linear_speed_limit') \
            .tooltip(f'Forward speed limit in m/s (default: {self.LINEAR_SPEED_LIMIT:.2f})')


def is_reference_valid(gnss: Gnss | None, *, max_distance: float = 5000.0) -> bool:
    if gnss is None:
        return True
    if GeoReference.current is None:
        return False
    if gnss.last_measurement is None:
        return False
    if gnss.last_measurement.gps_quality == 0:
        return False
    return gnss.last_measurement.point.distance(GeoReference.current.origin) <= max_distance
