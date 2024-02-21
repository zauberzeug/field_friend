import rosys
from nicegui import ui
from rosys.automation import Automator
from pyquaternion import Quaternion
import numpy as np
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
                r = ui.number(label='roll',value=0, step=1)
                p = ui.number(label='pitch',value=0, step=1)
                y = ui.number(label='yaw',value=0, step=1)

                ui.button(text='set IMU', on_click=lambda:field_friend.imu.simulate_measurement(rosys.geometry.Rotation.from_euler(roll=np.radians(r.value),pitch=np.radians(p.value),yaw=np.radians(y.value))))
                ui.button(text='imu emit', on_click=field_friend.imu.emit_measurement)
