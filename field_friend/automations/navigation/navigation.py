import abc
import logging
from typing import Any, Callable

import rosys


class Navigation(rosys.persistence.PersistentModule):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')

    @abc.abstractmethod
    async def on_start(self) -> bool:
        """Executed when the automation is started.

        Returns False if automation can not be started."""

    @abc.abstractmethod
    def clear(self) -> None:
        """Resets the state to initial configuration"""

    @abc.abstractmethod
    async def advance(self, condition: Callable[..., Any]) -> None:
        """Executed regularly when the automation is running to drive forward until condition is meat."""
