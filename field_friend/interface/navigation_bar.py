import os

from nicegui import ui

from ..hardware import FieldFriend, FieldFriendHardware


def navigation_bar(field_friend: FieldFriend):
    with ui.header().props('elevated').classes('q-pa-xs q-pt-sm', remove='q-pa-md items-start gap-4'):
        ui.label('Zauberzeug Field Friend').classes('text-white uppercase text-weight-bold col-5 q-pl-md mt-1')
        with ui.row().classes('col-7 justify-end q-pr-md items-bottom'):
            with ui.row().bind_visibility_from(field_friend.estop, 'active'):
                ui.icon('report').classes('text-red').props('size=sm dense unelevated flat')
                ui.label('emergency stop is pressed').classes('text-red mt-1')
            if isinstance(field_friend, FieldFriendHardware):
                ui.icon('hardware').classes('text-white').props('size=sm dense unelevated flat')
                ui.label('Hardware').classes('text-white mt-1')
            else:
                ui.icon('computer').classes('text-white').props('size=sm dense unelevated flat')
                ui.label('Simulation').classes('text-white mt-1')
            ui.icon('battery_full').classes('text-white').props('size=sm dense unelevated flat')
            battery_status = ui.markdown().classes('text-white mt-1')
            with ui.row():
                with ui.menu() as menu:
                    ui.menu_item('restart RoSys', on_click=lambda: os.utime('main.py'))
                ui.button(on_click=menu.open).classes('text-white').props('icon=settings size=sm dense unelevated flat')

    ui.timer(1, lambda: battery_status.set_content(f' {field_friend.bms.state.short_string}'))
