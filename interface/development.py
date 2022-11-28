import rosys
from nicegui import ui

import hardware


def development(robot: hardware.Robot, automator: rosys.automation.automator) -> None:
    with ui.card():
        if robot.is_real:
            with ui.row():
                with ui.column():
                    robot.robot_brain.developer_ui()
                with ui.column():
                    robot.robot_brain.communication.debug_ui()
            with ui.column():
                ui.markdown('**Axis Settings**').classes('col-grow')
                with ui.row():
                    with ui.menu() as developer_menu:
                        async def try_axis_home():
                            if not await robot.start_homing():
                                rosys.notify('homing: failed')
                            else:
                                rosys.notify('homing successful')
                        ui.menu_item('perform homing', on_click=lambda: automator.start(try_axis_home()))
                        ui.menu_item('Disable end stops', on_click=lambda: automator.start(
                            robot.enable_end_stops(False)))
                        ui.menu_item('Enable end stops', on_click=lambda: automator.start(robot.enable_end_stops(True)))
                    ui.button(on_click=developer_menu.open).props('dense fab-mini outline icon=more_vert')
                    robot_status = ui.markdown()
            ui.timer(1, lambda: robot_status.set_content(
                f' YAXIS: Alarm: {robot.yaxis_alarm} | Idle: {robot.yaxis_idle} | Pos: {robot.yaxis_position} | Home: {robot.yaxis_home_position} | Ref:{robot.yaxis_is_referenced} | endL: {robot.yaxis_end_l} | endR: {robot.yaxis_end_r}<br>'
                f'ZAXIS: Alarm: {robot.zaxis_alarm} | Idle: {robot.zaxis_idle} | Pos: {robot.zaxis_position} | Home: {robot.zaxis_home_position} | Ref:{robot.zaxis_is_referenced} | endT: {robot.zaxis_end_t} | endB: {robot.zaxis_end_b}'
            ))
        else:
            rosys.simulation_ui()
