from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from .manual_steerer_dialog import manual_steerer_dialog

if TYPE_CHECKING:
    from field_friend.system import System


class header_bar:
    def __init__(self, system: 'System', right_drawer: ui.right_drawer):
        self.drawer_icon = "expand_more"
        self.toggled = False
        self.STATUS_DRAWER_TOGGLED = rosys.event.Event()
        "tells if the status drawer is toggled or not."

        with ui.header().classes('items-center'):
            with ui.link(target='/'):
                ui.image('assets/zz_logo.png').tailwind.width('12')
            if system.field_friend.version in ['u1', 'u2']:
                ui.link('UCKERBOT', '/').classes('text-2xl text-white !no-underline mr-auto')
            else:
                ui.link('FIELD FRIEND', '/').classes('text-2xl text-white !no-underline mr-auto')

            with ui.row().bind_visibility_from(system.field_friend.estop, 'active').classes('mr-auto bg-red-500 text-white p-2 rounded-md'):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label('Emergency stop is pressed!').classes(
                    'text-white text-3xl').props('elevated')

            with ui.row().bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active').classes('mr-auto bg-red-500 rounded-md p-1'):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label('Software ESTOP is active!').classes('text-white text-3xl').props('elevated')

            with ui.row():
                ui.link('Field planner', '/field').classes('text-white text-lg !no-underline')
                ui.link('Path planner', '/path').classes('text-white text-lg !no-underline')
                ui.link('Development', '/dev').classes('text-white text-lg !no-underline')
            rosys.system.wifi_button().tooltip('add wifi connection').props('elevated')
            ui.button('Manual Steering', on_click=lambda system=system: manual_steerer_dialog(system)).tooltip(
                'Open the manual steering window to move the robot with a joystick.')
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

            def handle_toggle() -> None:
                right_drawer.toggle()
                self.toggled = not self.toggled
                self.STATUS_DRAWER_TOGGLED.emit()

            @ui.refreshable
            def change_drawer_icon() -> None:
                if not self.toggled:
                    self.drawer_icon = "expand_more"
                else:
                    self.drawer_icon = "chevron_right"
                ui.button(on_click=handle_toggle).props(f'icon={self.drawer_icon} flat color=white')
            change_drawer_icon()
            self.STATUS_DRAWER_TOGGLED.register(change_drawer_icon.refresh)
