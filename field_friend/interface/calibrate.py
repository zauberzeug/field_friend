import rosys
from nicegui import events, ui
from ..automations import Rolling
from ..hardware import IMU

class calibrate:

    def __init__(self,rolling: Rolling, imu: IMU) -> None:
        self.rolling = rolling 
        self.imu = imu
        with ui.card():
            with ui.row():
                self.left = ui.button(text= 'left roll: ' + str(self.rolling.left_roll_limit),on_click=lambda : self.rolling.set_roll_angle_left(left = self.imu.roll))
                self.right = ui.button(text= 'right roll: ' + str(self.rolling.right_roll_limit),on_click=lambda : self.rolling.set_roll_angle_rigth(rigth = self.imu.roll))
                self.forward = ui.button(text= 'forward pitch: ' + str(self.rolling.forward_pitch_limit),on_click=lambda : self.rolling.set_pitch_angle_forward(forward = self.imu.pitch))
                self.backward = ui.button(text= 'backward pitch. ' + str(self.rolling.backward_pitch_limit),on_click=lambda : self.rolling.set_pitch_angle_backward(backward = self.imu.pitch))
        ui.timer(rosys.config.ui_update_interval, self.update_status())
        

    def update_status(self) -> None :
        self.left.text =  'left roll: ' + str(self.rolling.left_roll_limit)
        self.right.text= 'right roll: ' + str(self.rolling.right_roll_limit)
        self.forward.text= 'forward pitch: ' + str(self.rolling.forward_pitch_limit)
        self.backward.text= 'backward pitch. ' + str(self.rolling.backward_pitch_limit)