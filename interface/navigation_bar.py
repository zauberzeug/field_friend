import os

from nicegui import ui

import hardware


def navigation_bar(robot: hardware.Robot):
    with ui.header().props('elevated').classes('q-pa-xs q-pt-sm', remove='q-pa-md items-start gap-4'):
        ui.label('Zauberzeug Field Friend').classes('text-white uppercase text-weight-bold col-5 q-pl-md mt-1')
        with ui.row().classes('col-7 justify-end q-pr-md items-bottom'):
            with ui.row().bind_visibility_from(robot, 'emergency_stop'):
                ui.icon('report').classes('text-white')
                ui.label('emergency halt is pressed').classes('text-white qmt-3')
            if isinstance(robot, hardware.RobotHardware):
                ui.icon('hardware').classes('text-white')
                ui.label('Hardware').classes('text-white mt-1')
            else:
                ui.icon('computer').classes('text-white')
                ui.label('Simulation').classes('text-white mt-1')
            ui.icon('battery_full').classes('text-white')
            battery_status = ui.markdown().classes('text-white mt-1')
            with ui.row():
                with ui.menu() as menu:
                    ui.menu_item('restart RoSys', on_click=lambda: os.utime('main.py'))
                ui.button(on_click=menu.open).classes('text-white').props('icon=settings size=sm dense unelevated flat')

    ui.timer(1, lambda: battery_status.set_content(f' {robot.battery.short_string}'))
