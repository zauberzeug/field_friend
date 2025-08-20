from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import aiohttp
import rosys
from nicegui import ui

if TYPE_CHECKING:
    from ..system import System


class DetectorHardware(rosys.vision.DetectorHardware):
    VERSION_CONTROL_MODES = Literal['auto', 'follow_loop', 'pause']

    def __init__(self, system: System, **kwargs):
        super().__init__(**kwargs)
        self.system = system
        self.bms = system.field_friend.bms

        self._version_control_mode: DetectorHardware.VERSION_CONTROL_MODES = 'auto'

        self.bms.CHARGING_STARTED.register(self._handle_charging)
        self.bms.CHARGING_STOPPED.register(self._handle_charging)
        rosys.on_startup(self._set_version_control_mode_on_startup)

    async def _handle_charging(self) -> None:
        if self._version_control_mode != 'auto':
            return
        await self.set_version_control_mode()

    async def _set_version_control_mode_on_startup(self) -> None:
        while not self.is_connected:
            await rosys.sleep(0.1)
        await self.set_version_control_mode()

    async def set_version_control_mode(self, *, new_mode: DetectorHardware.VERSION_CONTROL_MODES | None = None) -> None:
        version: Literal['follow_loop', 'pause']
        if new_mode is not None:
            if new_mode == self._version_control_mode:
                return
            self._version_control_mode = new_mode
        if self._version_control_mode == 'auto':
            version = 'follow_loop' if self.bms.state.is_charging else 'pause'
        else:
            version = self._version_control_mode
        await self.set_model_version(version)

    async def get_outbox_mode(self) -> bool | None:
        url = f'http://{self.host}:{self.port}/outbox_mode'
        async with aiohttp.request('GET', url) as response:
            if response.status != 200:
                self.log.error(f'Could not get outbox mode on port {self.port} - status code: {response.status}')
                return None
            response_text = await response.text()
        return response_text == 'continuous_upload'

    async def set_outbox_mode(self, value: bool) -> None:
        url = f'http://{self.host}:{self.port}/outbox_mode'
        async with aiohttp.request('PUT', url, data='continuous_upload' if value else 'stopped') as response:
            if response.status != 200:
                self.log.error(
                    f'Could not set outbox mode to {value} on port {self.port} - status code: {response.status}')
                return

    def developer_ui(self) -> None:
        ui.label('Detector').classes('text-center text-bold')

        with ui.column().classes('w-32'):
            options = {'follow_loop': 'Follow Loop', 'pause': 'Pause', 'auto': 'Auto'}
            ui.select(label='Version Control', options=options, value=self._version_control_mode,
                      on_change=lambda e: self.set_version_control_mode(new_mode=e.value)) \
                .classes('w-full') \
                .bind_value_from(self, '_version_control_mode') \
                .tooltip('Auto: Follow Loop if charging, Pause if not charging')
