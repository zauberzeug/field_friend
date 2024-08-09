import abc
import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from rosys.helpers import angle

from ..implements import Implement

if TYPE_CHECKING:
    from system import System


class WorkflowException(Exception):
    pass


class Navigation(rosys.persistence.PersistentModule):
    MAX_STRETCH_DISTANCE: float = 0.05
    DEFAULT_DRIVE_DISTANCE: float = 0.02
    LINEAR_SPEED_LIMIT: float = 0.13

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.driver = system.driver
        self.odometer = system.odometer
        self.gnss = system.gnss
        self.kpi_provider = system.kpi_provider
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.implement = implement
        self.detector = system.detector
        self.name = 'Unknown'
        self.start_position = self.odometer.prediction.point
        self.linear_speed_limit = self.LINEAR_SPEED_LIMIT
        self.angular_speed_limit = 0.1

    async def start(self) -> None:
        try:
            if not await self.implement.prepare():
                self.log.error('Tool-Preparation failed')
                return
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            self.start_position = self.odometer.prediction.point
            if isinstance(self.driver.wheels, rosys.hardware.WheelsSimulation) and not rosys.is_test:
                self.create_simulation()
            while not self._should_finish():
                await self.gnss.update_robot_pose()
                self.gnss.is_paused = True
                distance = await self.implement.get_stretch(self.MAX_STRETCH_DISTANCE)
                if distance > self.MAX_STRETCH_DISTANCE:  # we do not want to drive to long without observing
                    await self._drive(self.DEFAULT_DRIVE_DISTANCE)
                    self.gnss.is_paused = False
                    continue
                await self._drive(distance)
                self.gnss.is_paused = False
                await self.implement.start_workflow()
                await self.implement.stop_workflow()
        except WorkflowException as e:
            self.kpi_provider.increment_weeding_kpi('automation_stopped')
            self.log.error(f'WorkflowException: {e}')
        finally:
            self.kpi_provider.increment_weeding_kpi('weeding_completed')
            await self.implement.finish()
            await self.finish()
            await self.driver.wheels.stop()

    async def prepare(self) -> bool:
        """Prepares the navigation for the start of the automation

        Returns true if all preparations were successful, otherwise false."""
        self.plant_provider.clear()
        if isinstance(self.detector, rosys.vision.DetectorSimulation) and not rosys.is_test:
            self.detector.simulated_objects = []
        self.log.info('clearing plant provider')
        return True

    async def finish(self) -> None:
        """Executed after the navigation is done"""
        self.gnss.is_paused = False
        self.log.info('Navigation finished')

    @abc.abstractmethod
    async def _drive(self, distance: float) -> None:
        """Drives the vehicle a short distance forward"""

    async def _drive_to_yaw(self, distance: float, yaw: float, timeout: float = 2.0) -> None:
        deadline = rosys.time() + timeout
        start_position = self.odometer.prediction.point
        yaw = angle(self.odometer.prediction.yaw, yaw)  # take current yaw into account and only steer the difference
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, angular_speed_limit=self.angular_speed_limit):
            await self.driver.wheels.drive(*self.driver._throttle(1, yaw))  # pylint: disable=protected-access
        try:
            while self.odometer.prediction.point.distance(start_position) < distance:
                if rosys.time() >= deadline:
                    raise TimeoutError(
                        f'Driving Timeout at startpoint: {start_position} with yaw: {yaw} and target point: {self.odometer.prediction.point}')
                await rosys.sleep(0.01)
        finally:
            await self.driver.wheels.stop()

    @abc.abstractmethod
    def _should_finish(self) -> bool:
        """Returns True if the navigation should stop and be finished"""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup(self) -> dict:
        return {
            'linear_speed_limit': self.linear_speed_limit,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.linear_speed_limit = data.get('linear_speed_limit', self.linear_speed_limit)

    def create_simulation(self) -> None:
        pass

    def settings_ui(self) -> None:
        ui.number('Linear Speed', step=0.01, min=0.01, max=1.0, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'linear_speed_limit') \
            .tooltip(f'Forward speed limit in m/s (default: {self.LINEAR_SPEED_LIMIT:.2f})')
