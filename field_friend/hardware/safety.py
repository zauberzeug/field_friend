from rosys.hardware import Module, ModuleHardware, ModuleSimulation, RobotBrain, Wheels

from ..hardware import EStop


class Safety(Module):
    '''The safety module is a simple example for a representation of real or simulated robot hardware.
    '''

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)


class SafetyHardware(Safety, ModuleHardware):
    '''This module implements safety hardware.
    '''

    def __init__(self, robot_brain: RobotBrain, *,
                 wheels: Wheels,
                 estop: EStop) -> None:
        lizard_code = f'''
            let stop do {wheels.name}.stop(); end
            when {estop.name1}.level == 0 or {estop.name2}.level ==  0 then stop(); end
            when core.last_massage_age > 1000 then {wheels.name}.speed(0, 0); end'''
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def handle_core_output(self, time: float, words: list[str]) -> list[str]:
        return words


class SafetySimulation(Safety, ModuleSimulation):
    '''This module implements safety simulation.
    '''

    def __init__(self, wheels: Wheels, estop: EStop) -> None:
        super().__init__()
        self.wheels = wheels
        self.estop = estop

    async def step(self, dt: float) -> None:
        if self.estop.emergency_stop:
            self.wheels.stop()
