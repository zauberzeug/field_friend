import rosys
from nicegui import ui
from rosys.automation import Automator

from ..automations import Puncher
from ..hardware import FieldFriend, FieldFriendHardware


def development(field_friend: FieldFriend, automator: Automator, puncher: Puncher) -> None:
    with ui.card():
        if isinstance(field_friend, rosys.hardware.RobotHardware):
            with ui.row():
                with ui.column():
                    field_friend.robot_brain.developer_ui()
                with ui.column():
                    field_friend.robot_brain.communication.debug_ui()
        else:
            rosys.simulation_ui()
