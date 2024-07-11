import abc
import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from ..implements import Implement

if TYPE_CHECKING:
    from system import System


class WorkflowException(Exception):
    pass


class Navigation(rosys.persistence.PersistentModule):

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.driver = system.driver
        self.odometer = system.odometer
        self.kpi_provider = system.kpi_provider
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.implement = implement
        self.detector = system.detector
        self.name = 'Unknown'
        self.start_position = self.odometer.prediction.point

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
                max = 0.05
                distance = await self.implement.get_stretch(max)
                if distance > max:  # we do not want to drive to long without observing
                    await self._drive(0.02)
                    continue
                await self._drive(distance)
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

    @abc.abstractmethod
    async def _drive(self, distance: float) -> None:
        """Drives the vehicle forward

        This should only advance the robot by a small distance, e.g. 2 cm 
        to allow for adjustments and observations.
        """

    @abc.abstractmethod
    def _should_finish(self) -> bool:
        """Returns True if the navigation should stop and be finished"""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup(self) -> dict:
        return {}

    def restore(self, data: dict[str, Any]) -> None:
        pass

    def create_simulation(self) -> None:
        pass

    def settings_ui(self) -> None:
        pass
