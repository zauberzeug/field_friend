from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import rosys

if TYPE_CHECKING:
    from ...system import System


class Locator(rosys.persistence.PersistentModule):
    def __init__(self, system: System) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.locator')
        self.system = system

        self.is_paused = True

    def pause(self) -> None:
        if self.is_paused:
            return
        self.log.info('pausing locator')
        self.is_paused = True

    def resume(self) -> None:
        if not self.is_paused:
            return
        self.log.info('resuming locator')
        self.is_paused = False

    def developer_ui(self) -> None:
        pass

    def backup(self) -> dict:
        return {}

    def restore(self, data: dict[str, Any]) -> None:
        pass
