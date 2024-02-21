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
                r = ui.number(label='roll', step=1)
                p = ui.number(label='pitch', step=1)
                y = ui.number(label='yaw', step=1)

                ui.button(text='set IMU', on_click=field_friend.imu.simulate_measurement(rosys.geometry.Rotation.from_euler(roll=r,pitch=p,yaw=y)))
                ui.button(text='imu emit', on_click=field_friend.imu.emit_measurement())
