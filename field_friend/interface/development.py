import rosys
from nicegui import ui
from rosys.automation import Automator
import ast
from pyquaternion import Quaternion

from ..automations import Puncher
from ..hardware import FieldFriend, FieldFriendHardware


def development(field_friend: FieldFriend) -> None:
    with ui.card():
        if isinstance(field_friend, rosys.hardware.RobotHardware):
            with ui.row():
                with ui.column():
                    field_friend.robot_brain.developer_ui()
                with ui.column():
                    field_friend.robot_brain.communication.debug_ui()
        else:
            rosys.simulation_ui()
            with ui.card():
                w = ui.number(label='Quaternion w', value='0.0171122', step=0.01)
                i = ui.number(label='Quaternion i', value='0.7635265', step=0.01)
                j = ui.number(label='Quaternion j', value='-0.2304562', step=0.01)
                k = ui.number(label='Quaternion k', value='0.6030128', step=0.01)
                # field_friend.imu.set_quaternion(Quaternion(w.value, i.value, j.value, k.value))
                ui.button(text='set IMU', on_click=field_friend.imu.set_quaternion(
                    Quaternion(w.value, i.value, j.value, k.value)))
                ui.button(text='imu emit', on_click=field_friend.imu.emit_measurement())
