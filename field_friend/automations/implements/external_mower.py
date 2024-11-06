

import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from ...hardware import MowerHardware
from ..navigation import WorkflowException
from .implement import Implement

if TYPE_CHECKING:
    from ...system import System


class ExternalMower(Implement, rosys.persistence.PersistentModule):
    STRETCH_DISTANCE: float = 2.0

    def __init__(self, system: 'System') -> None:
        super().__init__('Mower')
        self.log = logging.getLogger('field_friend.mower')
        assert system.field_friend.mower is not None and isinstance(system.field_friend.mower, MowerHardware)
        self.mower_hardware: MowerHardware = system.field_friend.mower
        self.driver = system.driver
        assert self.driver is not None
        self.is_demo: bool = False
        self.stretch_distance: float = self.STRETCH_DISTANCE

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
            return min(self.stretch_distance, max_distance)
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
            'stretch_distance': self.stretch_distance,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.is_demo = data.get('is_demo', self.is_demo)
        self.stretch_distance = data.get('stretch_distance', self.stretch_distance)

    def settings_ui(self):
        ui.checkbox('Demo Mode', on_change=self.request_backup) \
            .bind_value(self, 'is_demo') \
            .tooltip('Do not start the mowing motors')
        ui.number('Stretch Distance', step=0.05, min=0.05, max=100.0, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'stretch_distance') \
            .tooltip(f'Forward speed limit in m/s (default: {self.STRETCH_DISTANCE:.2f})')
