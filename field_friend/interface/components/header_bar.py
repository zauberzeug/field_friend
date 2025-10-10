from __future__ import annotations

from typing import TYPE_CHECKING

import rosys
from nicegui import Event, ui

from ...hardware import FieldFriend
from .manual_steerer_dialog import ManualSteererDialog as manual_steerer_dialog
from .status_drawer import create_status_drawer

if TYPE_CHECKING:
    from ...system import System


def create_header(system: System) -> None:
    ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
    drawer = create_status_drawer(system)
    HeaderBar(system, drawer)


class HeaderBar:
    def __init__(self, system: System, right_drawer: ui.right_drawer):
        self.system = system
        self.drawer_icon = 'expand_more'
        self.toggled = False
        self.STATUS_DRAWER_TOGGLED: Event = Event()
        '''tells if the status drawer is toggled or not.'''

        with ui.header().classes('items-center py-3'):
            with ui.link(target='/'):
                ui.image('assets/zz_logo.png').classes('w-12')
            ui.link('FIELD FRIEND', '/').classes('text-2xl text-white !no-underline mr-auto')

            with ui.row().classes('mr-auto bg-red-500 text-white p-2 rounded-md') \
                .bind_visibility_from(system.field_friend.estop, 'active',
                                      backward=lambda active: active and not system.field_friend.estop.is_soft_estop_active):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label().bind_text_from(system.field_friend.estop, 'pressed_estops',
                                          lambda e: f'Emergency stop {e} is pressed!') \
                    .classes('text-white text-xl').props('elevated')

            with ui.row().bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active').classes('mr-auto bg-red-500 rounded-md p-1'):
                ui.icon('report').props('size=md').classes('text-white').props('elevated')
                ui.label('Software ESTOP is active!').classes('text-white text-3xl').props('elevated')

            with ui.row():
                # ui.link('Field planner', '/field').classes('text-white text-lg !no-underline')
                ui.link('Circle Sight', '/monitor').classes('text-white text-lg !no-underline')
                ui.link('Low Bandwidth', '/lb').classes('text-white text-lg !no-underline')
                ui.link('KPI', '/kpis').classes('text-white text-lg !no-underline')

            ui.button('Manual Steering', on_click=lambda system=system: manual_steerer_dialog(system)).tooltip(
                'Open the manual steering window to move the robot with a joystick.')

            self.internet_status = ui.icon('wifi', size='sm')
            if not rosys.is_simulation():
                self._update_internet_status()
                self.system.teltonika_router.CONNECTION_CHANGED.subscribe(self._update_internet_status)

            self._show_battery(system.field_friend)

            def handle_toggle() -> None:
                right_drawer.toggle()
                self.toggled = not self.toggled
                self.STATUS_DRAWER_TOGGLED.emit()

            @ui.refreshable
            def change_drawer_icon() -> None:
                if not self.toggled:
                    self.drawer_icon = 'expand_more'
                else:
                    self.drawer_icon = 'chevron_right'
                ui.button(on_click=handle_toggle).props(f'icon={self.drawer_icon} flat color=white')
            change_drawer_icon()
            self.STATUS_DRAWER_TOGGLED.subscribe(change_drawer_icon.refresh)

    def _show_battery(self, robot: FieldFriend) -> ui.row:
        with ui.row().classes('items-center gap-1') as row:
            ui.icon('battery_charging_full', size='sm').bind_visibility_from(robot.bms.state, 'is_charging')
            ui.icon('', size='sm').bind_name_from(robot.bms.state, 'percentage', lambda p: (
                'battery_unknown' if p is None else
                'battery_0_bar' if p < 5 else
                'battery_1_bar' if p < 20 else
                'battery_2_bar' if p < 35 else
                'battery_3_bar' if p < 50 else
                'battery_4_bar' if p < 65 else
                'battery_5_bar' if p < 80 else
                'battery_6_bar' if p < 95 else
                'battery_full'
            )).bind_visibility_from(robot.bms.state, 'is_charging', lambda c: not c)
            ui.label().bind_text_from(robot.bms.state, 'percentage',
                                      lambda p: f'{p:.0f}%' if p is not None else '?')
        return row

    def _update_internet_status(self) -> None:
        if self.system.teltonika_router.current_connection == 'ether':
            self.internet_status.name = 'cable'
            self.internet_status.props('size=sm')
        elif self.system.teltonika_router.current_connection == 'wifi':
            self.internet_status.name = 'wifi'
            self.internet_status.props('size=sm')
        elif self.system.teltonika_router.current_connection == 'mobile':
            self.internet_status.name = 'lte_mobiledata'
            self.internet_status.props('size=lg')
        else:
            self.internet_status.name = 'wifi_off'
            self.internet_status.props('size=sm')
