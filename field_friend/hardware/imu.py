import rosys


class IMUHardware(rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain) -> None:
        lizard_code = f'imu = Imu()'
        super().__init__(robot_brain, lizard_code=lizard_code)
