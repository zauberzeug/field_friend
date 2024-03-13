import rosys
from nicegui import ui

from ...hardware import FieldFriend


def development(field_friend: FieldFriend) -> None:
    with ui.card().style('background-color: #3E63A6; color: white;'):
        if isinstance(field_friend, rosys.hardware.RobotHardware):
            with ui.row():
                with ui.column():
                    field_friend.robot_brain.developer_ui()
                with ui.column():
                    field_friend.robot_brain.communication.debug_ui()
        else:
            rosys.simulation_ui()
