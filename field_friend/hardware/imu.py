import rosys
import abc

class IMU(rosys.hardware.Module, abc.ABC):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

class IMUHardware(IMU, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'imu') -> None:
        lizard_code = f'{name} = Imu()'
        super().__init__(robot_brain, lizard_code=lizard_code)

    def handle_core_output(self):
        return
    
    
class IMUSimulation(IMU, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'imu') -> None:
        self.name = name
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        super().__init__()
    
    async def set_euler_angles(self, yaw : float, pitch : float, roll : float):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll


