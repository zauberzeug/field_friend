import rosys
from nicegui import ui
from rosys.automation import Automator

from ..automations import Puncher
from ..hardware import FieldFriend, FieldFriendHardware


def development(field_friend: FieldFriend, automator: Automator, puncher: Puncher) -> None:
    with ui.card():
        if isinstance(field_friend, FieldFriendHardware):
            with ui.row():
                with ui.column():
                    field_friend.robot_brain.developer_ui()
                with ui.column():
                    field_friend.robot_brain.communication.debug_ui()
        else:
            rosys.simulation_ui()
        with ui.column():
            ui.markdown('**Axis Settings**').classes('col-grow')
            with ui.row():
                with ui.menu() as developer_menu:
                    async def try_axis_home():
                        await puncher.home()
                    ui.menu_item('perform homing', on_click=lambda: automator.start(try_axis_home()))
                    ui.menu_item('Disable end stops', on_click=lambda: automator.start(
                        field_friend.y_axis.enable_end_stops(False)))
                    ui.menu_item('Enable end stops', on_click=lambda: automator.start(
                        field_friend.z_axis.enable_end_stops(True)))
                ui.button(on_click=developer_menu.open).props('dense fab-mini outline icon=more_vert')
                robot_status = ui.markdown()
        ui.timer(1, lambda: robot_status.set_content(
            f' YAXIS: Alarm: {field_friend.y_axis.yaxis_alarm} | Idle: {field_friend.y_axis.yaxis_idle} | Pos: {field_friend.y_axis.yaxis_position} | Home: {field_friend.y_axis.yaxis_home_position} | Ref:{field_friend.y_axis.yaxis_is_referenced} | endL: {field_friend.y_axis.yaxis_end_l} | endR: {field_friend.y_axis.yaxis_end_r}<br>'
            f'ZAXIS: Alarm: {field_friend.z_axis.zaxis_alarm} | Idle: {field_friend.z_axis.zaxis_idle} | Pos: {field_friend.z_axis.zaxis_position} | Home: {field_friend.z_axis.zaxis_home_position} | Ref:{field_friend.z_axis.zaxis_is_referenced} | endT: {field_friend.z_axis.zaxis_end_t} | endB: {field_friend.z_axis.zaxis_end_b}'
        ))
