from __future__ import annotations

from typing import TYPE_CHECKING

import rosys
from nicegui import ui

if TYPE_CHECKING:
    from ...system import System


def create_settings_ui(system: System) -> None:
    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.label('Settings').classes('w-full text-center font-bold')
        ui.separator()
        with ui.column().classes('items-stretch'):
            if not rosys.is_simulation():
                ui.label('Learning Loop').classes('font-bold')
                with ui.row().classes('items-center'):
                    ui.switch('allow image upload via mobile network', value=False,
                              on_change=system.teltonika_router.MOBILE_UPLOAD_PERMISSION_CHANGED.emit) \
                        .tooltip('enable detected image upload to learning loop via mobile network. When off, '
                                 'images will only be uploaded with Wifi connection or cable') \
                        .bind_value(system.teltonika_router, 'mobile_upload_permission')
