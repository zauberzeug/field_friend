

from typing import TYPE_CHECKING

from ...hardware import MowerHardware
from .weeding_implement import Implement

if TYPE_CHECKING:
    from system import System


class ExternalMower(Implement):
    def __init__(self, system: 'System') -> None:
        super().__init__('Mower')
        self.system = system
        self.mower_hardware: MowerHardware = system.field_friend.mower
        assert self.mower_hardware is not None

    async def activate(self):
        await self.mower_hardware.turn_on()
        await super().activate()

    async def deactivate(self):
        await self.mower_hardware.turn_off()
        await super().deactivate()
