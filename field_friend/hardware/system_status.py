import datetime
import os
from pathlib import Path
from typing import TYPE_CHECKING

import psutil
import rosys
from nicegui import ui

if TYPE_CHECKING:
    from system import System


def system_status(system: 'System') -> None:
    with ui.column().classes('gap-1 text-xs text-gray-500'):
        line1 = ui.label()
        line2 = ui.markdown().classes('mb-[-0.5rem]')
        line3 = ui.markdown().classes('mb-[-0.5rem]')

    async def update_status() -> None:
        line1.set_text(
            f'Uptime: {datetime.timedelta(seconds=rosys.uptime())}, '
            f'Battery: {system.field_friend.bms.state.short_string}, '
            f'CPU: {psutil.cpu_percent():.0f} % '
        )
        line2.set_content(
            f'Position: {system.gnss.current}, '
        )
        bumpers = system.field_friend.bumper.active_bumpers
        status_flags: list[str] = [
            'Paused' if system.automator.is_paused else 'Running' if system.automator.is_running else '',
            f'**E Stop**' if system.field_friend.estop.active else '',
            f'**Bump:** ' + ' '.join(bumpers) if bumpers else '',
            f'**Charging** ' if system.field_friend.bms.state.is_charging else '',
        ]
        line3.set_content(', '.join(flag.strip() for flag in status_flags if flag))

    ui.timer(rosys.config.ui_update_interval, update_status)
