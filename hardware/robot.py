import rosys

from . import zauberzeug


class Robot(zauberzeug.Robot):
    pass


class RobotHardware(zauberzeug.RobotHardware, Robot):

    def __init__(self) -> None:
        super().__init__()


class RobotSimulation(zauberzeug.RobotSimulation, Robot):
    pass
