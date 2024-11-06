from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, cast

from nicegui import background_tasks, ui
from nicegui.version import __version__ as nicegui_version
from rosys.version import __version__ as rosys_version

from ...hardware import FieldFriendHardware

if TYPE_CHECKING:
    from ...system import System

MUTEX_PATH = Path('.livesync_mutex')


class Info:

    def __init__(self, system: System) -> None:
        self.system = system

    def create_dialog(self) -> ui.dialog:
        with ui.dialog() as dialog, ui.card():
            ui.label('Versions').classes('text-xl font-semibold')
            with ui.grid(columns=2):
                # ui.label('Fieldfriend:')
                # ui.label(self.system.version)
                ui.label('RoSys:')
                ui.label(rosys_version)
                ui.label('NiceGUI:')
                ui.label(nicegui_version)

                if self.system.is_real:
                    lizard_firmware = cast(FieldFriendHardware, self.system.field_friend).robot_brain.lizard_firmware

                    async def read_lizard_versions() -> None:
                        await lizard_firmware.read_core_version()
                        await lizard_firmware.read_p0_version()
                    background_tasks.create(read_lizard_versions())

                    ui.label('Lizard (Core):')
                    ui.label().bind_text_from(lizard_firmware, 'core_version')
                    ui.label('Lizard (P0):')
                    ui.label().bind_text_from(lizard_firmware, 'p0_version')

            if MUTEX_PATH.exists():
                ui.label('Livesync').classes('text-xl font-semibold')
                ui.code(MUTEX_PATH.read_text().strip(), language=None)

        return dialog
