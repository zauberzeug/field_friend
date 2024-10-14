from typing import TYPE_CHECKING

import rosys
from nicegui import ui

if TYPE_CHECKING:
    from system import System


def bms_status(system: 'System') -> None:
    with ui.column().classes('gap-1 text-xs text-gray-500'):
        line1 = ui.label()

    async def update_status() -> None:
        line1.set_text(
            f'Current Battery Status: {system.field_friend.bms.state.short_string}'
        )
    ui.timer(rosys.config.ui_update_interval, update_status)
