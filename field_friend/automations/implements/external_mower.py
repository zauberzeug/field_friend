

import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from ...hardware import MowerHardware
from ..navigation import WorkflowException
from .weeding_implement import Implement

if TYPE_CHECKING:
    from system import System


class ExternalMower(Implement, rosys.persistence.PersistentModule):
    def __init__(self, system: 'System') -> None:
        super().__init__('Mower')
        self.log = logging.getLogger('field_friend.mower')
        self.mower_hardware: MowerHardware = system.field_friend.mower
        self.driver = system.driver
        assert self.mower_hardware is not None
        assert self.driver is not None
        self.is_demo: bool = False

    async def activate(self):
        if not self.is_demo:
            await self.mower_hardware.turn_on()
            await rosys.sleep(2)
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
        self.log.warning('Stuck motor detected')
        await self.driver.wheels.stop()
        await self.mower_hardware.reset_motors()
        await rosys.sleep(5)
        await self.mower_hardware.turn_on()
        await rosys.sleep(2)
        return 0.0

    def backup(self) -> dict:
        return {
            'is_demo': self.is_demo,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.is_demo = data.get('is_demo', self.is_demo)

    def settings_ui(self):
        ui.checkbox('Demo Mode') \
            .bind_value(self, 'is_demo') \
            .tooltip('Do not start the mowing motors')
