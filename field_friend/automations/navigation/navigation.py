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
        self.name = 'Unknown'
        self.start_position = self.odometer.prediction.point

        self.return_to_start: bool = True

    async def start(self) -> None:
        try:
            if isinstance(self.driver.wheels, rosys.hardware.WheelsSimulation) and not rosys.is_test:
                self.create_simulation()
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            self.start_position = self.odometer.prediction.point
            if not await self.implement.prepare():
                self.log.error('Tool-Preparation failed')
                return
            while not self._should_finish():
                await rosys.automation.parallelize(
                    self.implement.observe(),
                    self._proceed(),
                    return_when_first_completed=True
                )
                if not self._should_finish():
                    await self.implement.start_workflow()
                    await self.implement.stop_workflow()
                    await self._drive()
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
        return True

    async def finish(self) -> None:
        """Executed after the navigation is done"""

    async def _proceed(self):
        while not self._should_finish():
            await self._drive()

    @abc.abstractmethod
    async def _drive(self) -> None:
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
        return {
            'return_to_start': self.return_to_start,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.return_to_start = data.get('return_to_start', self.return_to_start)

    def create_simulation(self) -> None:
        pass

    def settings_ui(self) -> None:
        ui.checkbox('Return to start position', value=True).bind_value(self, 'return_to_start') \
            .tooltip('Set the weeding automation to drive backwards to the start row at the end of the row')
