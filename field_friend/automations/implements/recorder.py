

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

    async def prepare(self) -> bool:
        return True

    async def finish(self) -> None:
        pass

    async def activate(self):
        self.system.plant_locator.pause()
        self.system.plant_provider.clear()
        await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)  # NOTE: we wait for the camera to adjust
        self.system.plant_locator.resume()

    async def deactivate(self):
        self.system.plant_locator.pause()

    async def observe(self) -> None:
        while True:
            await rosys.sleep(0.5)

    async def on_focus(self) -> None:
        pass

    def settings_ui(self):
        pass

    def reset_kpis(self):
        pass
