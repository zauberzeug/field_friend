import abc
import logging
from typing import Any

import rosys

from ..tool.tool import Tool


class Navigation(rosys.persistence.PersistentModule):

    def __init__(self,
                 driver: rosys.driving.Driver,
                 odometer: rosys.driving.Odometer,
                 tool: Tool,
                 ) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.driver = driver
        self.odometer = odometer
        self.tool = tool

    @abc.abstractmethod
    async def start(self) -> None:
        """Executed to start the automation.

        Returns False if automation can not be started."""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup(self) -> dict:
        return {}

    def restore(self, data: dict[str, Any]) -> None:
        pass
