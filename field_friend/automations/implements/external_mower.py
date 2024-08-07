

import logging
from typing import TYPE_CHECKING

import rosys

from ...hardware import MowerHardware
from ..navigation import WorkflowException
from .weeding_implement import Implement

if TYPE_CHECKING:
    from system import System


class ExternalMower(Implement):
    def __init__(self, system: 'System') -> None:
        super().__init__('Mower')
        self.log = logging.getLogger('field_friend.mower')
        self.mower_hardware: MowerHardware = system.field_friend.mower
        self.driver = system.driver
        assert self.mower_hardware is not None
        assert self.driver is not None

    async def activate(self):
        await self.mower_hardware.turn_on()
        await super().activate()

    async def deactivate(self):
        await self.mower_hardware.turn_off()
        await super().deactivate()

    async def get_stretch(self, max_distance: float) -> float:
        if not any([self.mower_hardware.m0_error, self.mower_hardware.m1_error, self.mower_hardware.m2_error]):
            return 0.02
        if all([self.mower_hardware.m0_error, self.mower_hardware.m1_error, self.mower_hardware.m2_error]):
            rosys.notify('All motors are stuck', 'negative')
            raise WorkflowException('All motors are stuck')
        # TODO: implement a better error handling
        await rosys.sleep(0.1)
        self.log.error('Stuck motor detected')
        await self.mower_hardware.turn_off()
        await rosys.sleep(2)
        await self.driver.wheels.drive(*self.driver._throttle(-0.1, 0.0))
        await rosys.sleep(2)
        await self.driver.wheels.stop()
        await rosys.sleep(1)
        await self.mower_hardware.reset_motors()
        await rosys.sleep(5)
        await self.mower_hardware.turn_on()
        return 0.0
