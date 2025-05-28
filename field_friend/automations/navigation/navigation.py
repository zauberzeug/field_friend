from __future__ import annotations

import abc
import logging
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
import rosys.geometry
from nicegui import ui
from rosys.analysis import track
from rosys.geometry import GeoReference, Point, Pose
from rosys.hardware import Gnss
from rosys.helpers import ramp

from ...hardware import DoubleWheelsHardware, WheelsSimulationWithAcceleration
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
        self.angular_speed_limit = 0.1
        self.throttle_at_end = isinstance(self.driver.wheels, DoubleWheelsHardware | WheelsSimulationWithAcceleration)

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
                    self.log.error('Getting move target')
                    move_target = await self.implement.get_move_target()
                    if not move_target:
                        self.log.warning('No move target found')
                        break
                    move_pose = Pose(x=move_target.x, y=move_target.y, yaw=self.target_heading)
                    # TODO: using WORK_Y doesnt seem to work, we should check that
                    move_pose = move_pose.transform_pose(Pose(x=-self.system.field_friend.WORK_X, y=0, yaw=0))
                    self.log.debug('Moving from %s to %s', self.robot_locator.pose, move_pose)
                    await self._drive_to_target(move_pose, throttle_at_end=self.throttle_at_end)
                    await self.driver.wheels.stop()
                    self.log.debug('Stopped at %s to weed, %s', self.robot_locator.pose, move_pose)
                    await self.implement.start_workflow()
                    await self.implement.stop_workflow()
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

    @abc.abstractmethod
    async def _drive(self) -> None:
        """Drive the robot to the next waypoint of the navigation"""

    @property
    def is_moving(self) -> bool:
        return self.robot_locator._wheels.linear_target_speed > 0.005 or self.robot_locator._wheels.angular_target_speed > 0.01

    @track
    async def _drive_to_target(self, target: Pose, *, max_turn_angle: float = 0.1, throttle_at_end: bool = True, predicted_deceleration: float = 0.3, interval: float = 0.05) -> None:
        """
        Drive the robot to the target position with a limited turn angle to avoid too harsh turns.
        Consider using using rosys.driving.Driver.drive_spline if you need precise trajectory following.

        :param target: The target pose to drive to.
        :param max_turn_angle: The maximum turn angle in radians.
        :param throttle_at_end: Whether to throttle down to stop precisely at the target position.
        :param predicted_deceleration: The predicted deceleration of the robot. The braking distance is calculated with this value.
        """
        relative_distance = self.robot_locator.pose.relative_point(target.point).x
        total_distance = self.robot_locator.pose.distance(target)
        self.log.debug('Driving to target: %s for %sm with relative_distance: %s', target, total_distance, relative_distance)
        self.log.debug('is_moving: %s with linear: %.6f  --  wheels.is_moving: %s', self.robot_locator.is_moving, self.robot_locator.current_velocity.linear, self.is_moving)
        total_distance = relative_distance
        max_stop_distance: float | None = None
        max_braking_distance = self.linear_speed_limit**2 / (2 * predicted_deceleration)
        i = 0
        while True:
            relative_point = self.robot_locator.pose.relative_point(target.point)
            self.log.debug('relative_x: %s', relative_point.x)
            if relative_point.x <= 0:
                self.log.debug('Reached target: %s', self.robot_locator.pose)
                break
            # distance = self.robot_locator.pose.distance(target)
            # TODO: only for short distances
            distance = self.robot_locator.pose.relative_point(target.point).x

            hook_offset = Point(x=self.driver.parameters.hook_offset, y=0)
            carrot_offset = Point(x=self.driver.parameters.carrot_offset, y=0)
            target_point = target.transform(carrot_offset)
            hook = self.robot_locator.pose.transform(hook_offset)
            turn_angle = rosys.helpers.angle(self.robot_locator.pose.yaw, hook.direction(target_point))
            turn_angle = min(turn_angle, max_turn_angle)
            turn_angle = max(turn_angle, -max_turn_angle)
            curvature = np.tan(turn_angle) / hook_offset.x
            if curvature != 0 and abs(1 / curvature) < self.driver.parameters.minimum_turning_radius:
                curvature = (-1 if curvature < 0 else 1) / self.driver.parameters.minimum_turning_radius

            linear: float = self.linear_speed_limit
            now = rosys.time()
            if throttle_at_end:
                if self.robot_locator.is_moving:
                    brake_distance = self.robot_locator.current_velocity.linear**2 / (2 * predicted_deceleration)
                    self.log.debug('Brake distance: %s', brake_distance)
                else:
                    brake_distance = max_braking_distance
                    self.log.debug('Using max braking distance: %s', brake_distance)
                brake_point = target.transform(Point(x=-brake_distance, y=0))
                relative_brake_point = self.robot_locator.pose.relative_point(brake_point)
                self.log.debug('Relative brake point: %s', relative_brake_point.x)
                if relative_brake_point.x < 0:
                    if max_stop_distance is None:
                        max_stop_distance = brake_distance
                        self.log.debug('Setting max stop distance to %s', max_stop_distance)
                    ramp_factor = ramp(distance, 0.0, max_stop_distance, 0.01, 0.5, clip=True)
                    linear *= ramp_factor
                    self.log.debug('Decelerating: current=%s, time_diff=%s, brake_distance=%s, target_distance=%s, brake_point=%s, ramp_factor=%s, linear=%s, velocity=%s', self.robot_locator.pose, now - self.robot_locator.pose.time, brake_distance, distance, brake_point, ramp_factor, linear, self.robot_locator.current_velocity.linear)
                else:
                    self.log.debug('Driving: current=%s, time_diff=%s, brake_distance=%s, brake_point=%s, linear=%s, velocity=%s', self.robot_locator.pose, now - self.robot_locator.pose.time, brake_distance, brake_point, linear, self.robot_locator.current_velocity.linear)
            linear = max(linear, self.LINEAR_SPEED_MINIMUM)
            angular = linear * curvature

            # NOTE: use only a quarter of the speed to avoid undershooting the target
            distance_per_timestamp = min(linear * interval * 0.50, self.linear_speed_limit * interval)
            next_relative_x = relative_point.x - distance_per_timestamp
            self.log.debug('next_relative_x: %s', next_relative_x)
            if throttle_at_end and i > 0 and next_relative_x <= 0:
                self.log.debug('Will reach target in %ss', next_relative_x / distance_per_timestamp)
                break
            with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, angular_speed_limit=self.angular_speed_limit):
                await self.driver.wheels.drive(*self.driver._throttle(linear, angular))  # pylint: disable=protected-access
            await rosys.sleep(interval)
            i += 1
        await self.driver.wheels.stop()
        self.log.debug('Driving done at %s for target: %s', self.robot_locator.pose, target)
        while self.is_moving:
            await rosys.sleep(0.1)
        self.log.debug('Real stop at %s with error: %.2fmm', self.robot_locator.pose.x, (self.robot_locator.pose.x - target.x) * 1000)

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
        ui.number('Linear Speed', step=0.01, min=0.01, max=1.0, format='%.2f', on_change=self.request_backup) \
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
