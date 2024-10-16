

from typing import TYPE_CHECKING

import rosys

from .weeding_implement import Implement

if TYPE_CHECKING:
    from system import System


class Recorder(Implement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Recorder')
        self.system = system

    async def activate(self):
        self.system.plant_provider.clear()
        await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)  # NOTE: we wait for the camera to adjust
        self.system.plant_locator.resume()
        await super().activate()
    

    async def deactivate(self):
        self.system.plant_locator.pause()
        await self.system.field_friend.flashlight.turn_off()
        await super().deactivate()