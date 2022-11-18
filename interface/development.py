import rosys
from nicegui import ui

import hardware


def development(robot: hardware.robot.Robot, automator: rosys.automation.automator) -> None:
    with ui.card():
        if robot.is_real:
            robot.robot_brain.developer_ui()
            robot.robot_brain.communication.debug_ui()
            with ui.row():
                with ui.menu() as developer_menu:
                    ui.menu_item('perform homing', on_click=robot.start_homing)
                    ui.menu_item('Disable end stops', on_click=lambda: automator.start(robot.enable_end_stops(False)))
                    ui.menu_item('Enable end stops', on_click=lambda: automator.start(robot.enable_end_stops(True)))
                ui.button(on_click=developer_menu.open).props('dense fab-mini outline icon=more_vert')
                robot_status = ui.markdown()
            rosys.on_startup(robot.robot_brain.lizard_firmware.ensure_lizard_version)
            ui.timer(1, lambda: robot_status.set_content(
                f' yaxis: Alarm: {robot.yaxis_alarm} Idle: {robot.yaxis_idle} Pos: {robot.yaxis_position} Home: {robot.yaxis_home_position} Ref:{robot.yaxis_is_referenced}<br>'
                f'zaxis: Alarm: {robot.zaxis_alarm} Idle: {robot.zaxis_idle} Pos: {robot.zaxis_position} Home: {robot.zaxis_home_position} Ref:{robot.zaxis_is_referenced}'
            ))
        else:
            rosys.simulation_ui()
