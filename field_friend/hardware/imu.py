import rosys
import abc
from dataclasses import dataclass

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
                 name: str = 'imu') -> None:
        self.name = name
        self.acc_x = 0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        lizard_code = f'{name} = Imu()'
        core_message_fields = [f'{name}.acc_x',
                               f'{name}.roll',
                               f'{name}.pitch',
                               f'{name}.yaw',
                               ]
        super().__init__(robot_brain = robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)


    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.acc_x = float(words.pop(0))
        self.roll = float(words.pop(0))
        self.pitch = float(words.pop(0))
        self.yaw = float(words.pop(0))
        angles = Eulerangle(
            roll = self.roll,
            pitch = self.pitch,
            yaw = self.yaw)
        self.ROBOT_ANGLES.emit(angles)


class IMUSimulation(IMU, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'imu') -> None:
        self.name = name
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        self.acc_x = 0
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


