from typing import TYPE_CHECKING

import rosys
from nicegui import ui

if TYPE_CHECKING:
    from system import System


def header_bar(system: 'System', right_drawer: ui.right_drawer) -> None:
    with ui.header().classes('items-center'):
        ui.label('Zauberzeug Field Friend').classes('text-2xl mr-auto')

        rosys.system.wifi_button().tooltip('add wifi connection').props('elevated')
        with ui.button().props('icon=settings flat color=white'):
            with ui.menu().props(remove='no-parent-event'):
                with ui.column().classes('gap-0'):
                    rosys.persistence.export_button() \
                        .props('flat align=left').classes('w-full')
                    rosys.persistence.import_button(after_import=system.restart) \
                        .props('flat align=left').classes('w-full')
                ui.menu_item('Restart RoSys', on_click=system.restart)
                if system.is_real:
                    ui.menu_item('Restart Lizard', on_click=system.field_friend.robot_brain.restart)
                ui.menu_item('Clear GNSS reference', on_click=system.gnss.clear_reference)
        ui.button(on_click=right_drawer.toggle).props('icon=menu flat color=white')
