import abc
import logging
from typing import Any

import rosys

from ..kpi_provider import KpiProvider
from ..tool.tool import Implement


class WorkflowException(Exception):
    pass


class Navigation(rosys.persistence.PersistentModule):

    def __init__(self,
                 driver: rosys.driving.Driver,
                 odometer: rosys.driving.Odometer,
                 kpi_provider: KpiProvider,
                 implement: Implement,
                 ) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.driver = driver
        self.odometer = odometer
        self.kpi_provider = kpi_provider
        self.implement = implement

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
        return {}

    def restore(self, data: dict[str, Any]) -> None:
        pass
