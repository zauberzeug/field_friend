import rosys
import abc
from dataclasses import dataclass
import numpy as np

@dataclass(slots=True, kw_only=True)
class Eulerangle:
    roll: float
    pitch: float
    yaw: float


class IMU(rosys.hardware.Module, abc.ABC):
    def __init__(self, **kwargs) -> None:
        self.ROBOT_ANGLES = rosys.event.Event()
        super().__init__(**kwargs)

class IMUHardware(IMU, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'imu',pitch_offset: float, roll_offset: float) -> None:
        self.name = name
        self.pitch_offset = pitch_offset
        self.roll_offset = roll_offset
        self.acc_x = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.cal_sys = 0
        self.cal_gyro = 0
        self.cal_accel = 0
        self.cal_mag = 0
        
        lizard_code = f'{name} = Imu()'
        core_message_fields = [f'{name}.acc_x',
                               f'{name}.roll',
                               f'{name}.pitch',
                               f'{name}.yaw',
                               f'{name}.cal_sys',
                               f'{name}.cal_gyro',
                               f'{name}.cal_accel',
                               f'{name}.cal_mag',
                               ]
        super().__init__(robot_brain = robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)


    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.acc_x = float(words.pop(0))
        self.roll = float(words.pop(0))
        self.pitch = float(words.pop(0))
        self.yaw = float(words.pop(0))
        self.cal_sys = float(words.pop(0))
        self.cal_gyro = float(words.pop(0))
        self.cal_accel = float(words.pop(0))
        self.cal_mag = float(words.pop(0))

        self.offset_correction()

        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        
        if self.cal_gyro > 1.0:
            self.ROBOT_ANGLES.emit(angles)
            
    #TODO auslagern jetzt nur zum testen hier
    def offset_correction(self)->None:
        self.roll = self.roll + self.roll_offset
        self.pitch = self.pitch + self.pitch_offset




class IMUSimulation(IMU, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'imu') -> None:
        self.name = name
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.acc_x = 0
        self.cal_sys = 0
        self.cal_gyro = 0
        self.cal_accel = 0
        self.cal_mag = 0
        super().__init__()
    
    async def set_euler_angles(self, yaw : float, pitch : float, roll : float):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        self.ROBOT_ANGLES.emit(angles)
    
    async def testemit(self) -> None:
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        self.ROBOT_ANGLES.emit(angles)


