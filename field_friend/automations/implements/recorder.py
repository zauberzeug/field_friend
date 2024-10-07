

from typing import TYPE_CHECKING, Optional

import rosys

from .weeding_implement import Implement

if TYPE_CHECKING:
    from system import System


class Recorder(Implement):

    def __init__(self, system: 'System') -> None:
        super().__init__('Recorder')
        self.system = system
        self.kpi_provider = system.kpi_provider
        self.state: str = 'idle'
        self.start_time: Optional[float] = None
        rosys.on_repeat(self._update_time_and_distance, 0.1)

    async def activate(self):
        self.system.plant_provider.clear()
        await self.system.field_friend.flashlight.turn_on()
        await rosys.sleep(3)  # NOTE: we wait for the camera to adjust
        self.system.plant_locator.resume()
        await super().activate()
    
    async def prepare(self) -> bool:
        self.state = 'running'
        return True

    async def finish(self) -> None:
        self.state = 'idle'
        await super().finish()


    async def deactivate(self):
        self.system.plant_locator.pause()
        await self.system.field_friend.flashlight.turn_off()
        await super().deactivate()

    def _update_time_and_distance(self):
        # TODO move this to base class?
        if self.state == 'idle':
            return
        if self.start_time is None:
            self.start_time = rosys.time()
        passed_time = rosys.time() - self.start_time
        if passed_time > 1:
            self.kpi_provider.increment_all_time_kpi('time')
            self.start_time = rosys.time()
