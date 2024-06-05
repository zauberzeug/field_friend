

from typing import TYPE_CHECKING

import rosys

from ..puncher import PuncherException
from .weeding_implement import Implement, ImplementException

if TYPE_CHECKING:
    from system import System


class Recorder(Implement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Recorder')
        self.system = system

    async def activate(self):
        super().activate()
        self.system.plant_provider.clear()
        await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)  # NOTE: we wait for the camera to adjust
        self.system.plant_locator.resume()

    async def deactivate(self):
        self.system.plant_locator.pause()
        self.system.field_friend.flashlight.turn_off()
        super().deactivate()

    async def observe(self) -> None:
        while True:
            await rosys.sleep(0.5)
