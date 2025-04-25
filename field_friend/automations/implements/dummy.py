
from __future__ import annotations

from typing import TYPE_CHECKING

from .weeding_implement import Implement

if TYPE_CHECKING:
    from ...system import System


class Dummy(Implement):

    def __init__(self, system: System) -> None:
        super().__init__('Dummy')
        self.system = system

    async def prepare(self) -> bool:
        await super().prepare()
        return True

    async def activate(self):
        await super().activate()

    async def deactivate(self):
        await super().deactivate()
