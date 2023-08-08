from typing import TYPE_CHECKING

import rosys
from nicegui import ui

if TYPE_CHECKING:
    from system import System


def header_bar(system: 'System', right_drawer: ui.right_drawer) -> None:
    with ui.header().classes('items-center'):
        with ui.link(target='/'):
            ui.image('assets/zz_logo.png').tailwind.width('12')
        if system.field_friend.version in ['u1', 'u2']:
            ui.link('UCKERBOT', '/').classes('text-2xl text-white !no-underline mr-auto')
        else:
            ui.link('FIELD FRIEND', '/').classes('text-2xl text-white !no-underline mr-auto')

        with ui.row().bind_visibility_from(system.field_friend.estop, 'active').classes('mr-auto'):
            ui.icon('report').props('size=md').classes('text-red-400')
            ui.label('Emergency stop is pressed!').classes('text-red-400 text-2xl')

        with ui.row().bind_visibility_from(system.field_friend.estop, 'en3_active').classes('mr-auto'):
            ui.icon('report').props('size=md').classes('text-red-400')
            ui.label('Software ESTOP is active!').classes('text-red-400 text-2xl')

        with ui.row():
            ui.link('Field planner', '/field').classes('text-white text-lg !no-underline')
            ui.link('Path planner', '/path').classes('text-white text-lg !no-underline')
            ui.link('Development', '/dev').classes('text-white text-lg !no-underline')
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
