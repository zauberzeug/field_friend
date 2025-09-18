

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui

from .weeding_implement import Implement

if TYPE_CHECKING:
    from ...system import System


class Recorder(Implement):
    RECORDING_INTERVAL = 1.0

    def __init__(self, system: System) -> None:
        super().__init__('Recorder')
        self.system = system
        self.recording_interval = self.RECORDING_INTERVAL

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
        self.system.plant_locator.interval = self.recording_interval
        self.system.plant_locator.resume()
        await super().activate()

    async def deactivate(self):
        self.system.plant_locator.pause()
        self.system.plant_locator.interval = self.system.plant_locator.INTERVAL
        if self.system.field_friend.flashlight:
            await self.system.field_friend.flashlight.turn_off()
        await super().deactivate()

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'recording_interval': self.recording_interval,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.recording_interval = data.get('recording_interval', self.recording_interval)

    def settings_ui(self):
        super().settings_ui()
        ui.number('Recording interval', step=0.01, min=0.01, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined suffix=s') \
            .classes('w-28') \
            .bind_value(self, 'recording_interval') \
            .tooltip(f'Set the interval between image captures (default: {self.RECORDING_INTERVAL:.2f}s)')
