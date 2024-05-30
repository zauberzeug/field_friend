import abc
import logging
from typing import TYPE_CHECKING, Any

import rosys

from ..implements.implement import Implement

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
        self.implement = implement
        self.name = 'Unknown'

        self.drive_backwards_to_start: bool = True
        self.drive_to_start: bool = True

    async def start(self) -> None:
        try:
            await self._start()
        except WorkflowException as e:
            self.kpi_provider.increment_weeding_kpi('automation_stopped')
            self.log.error(f'WorkflowException: {e}')
        finally:
            self.kpi_provider.increment_weeding_kpi('weeding_completed')
            await self.implement.finish()
            await self.driver.wheels.stop()

    @abc.abstractmethod
    async def _start(self) -> None:
        """Executed to start the automation.

        Returns False if automation can not be started."""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup(self) -> dict:
        return {
            'drive_backwards_to_start': self.drive_backwards_to_start,
            'drive_to_start': self.drive_to_start,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.drive_backwards_to_start = data.get('drive_backwards_to_start', self.drive_backwards_to_start)
        self.drive_to_start = data.get('drive_to_start', self.drive_to_start)

    def create_simulation(self):
        pass
