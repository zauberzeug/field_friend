import rosys

from . import zauberzeug


class Robot(zauberzeug.Robot):
    pass


class RobotHardware(zauberzeug.RobotHardware, Robot):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain) -> None:
        super().__init__(robot_brain)


class RobotSimulation(zauberzeug.RobotSimulation, Robot):
    pass
