

from __future__ import annotations

from typing import TYPE_CHECKING

import rosys

from .weeding_implement import Implement

if TYPE_CHECKING:
    from ...system import System


class Recorder(Implement):

    def __init__(self, system: System) -> None:
        super().__init__('Recorder')
        self.system = system

    async def prepare(self) -> bool:
        await super().prepare()
        self.system.plant_provider.clear()
        if self.system.plant_locator.detector_info is None and not await self.system.plant_locator.fetch_detector_info():
            rosys.notify('Dectection model information not available', 'negative')
            return False
        return True

    async def activate(self):
        if self.system.field_friend.flashlight:
            await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)  # NOTE: we wait for the camera to adjust
        self.system.plant_locator.resume()
        await super().activate()

    async def deactivate(self):
        self.system.plant_locator.pause()
        if self.system.field_friend.flashlight:
            await self.system.field_friend.flashlight.turn_off()
        await super().deactivate()
