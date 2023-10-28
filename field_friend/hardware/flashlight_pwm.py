import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class FlashlightPWM(rosys.hardware.Module, abc.ABC):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    async def __aenter__(self):
        await self.turn_on()

    async def __aexit__(self, exc_t, exc_v, exc_tb):
        await self.turn_off()

    @abc.abstractmethod
    async def turn_on(self) -> None:
        pass

    @abc.abstractmethod
    async def turn_off(self) -> None:
        pass


class FlashlightPWMHardware(FlashlightPWM, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 name: str = 'flashlight',
                 front_pin: int = 12,
                 back_pin: int = 23,
                 duty_cycle: int = 0,
                 frequency: int = 0) -> None:
        self.name = name
        self.expander = expander
        self.duty_cycle = duty_cycle
        self.frequency = frequency
        lizard_code = remove_indentation(f'''
            {name}_front = {expander.name + "." if expander else ""}PwmOutput({front_pin})
            {name}_back = {expander.name + "." if expander else ""}PwmOutput({back_pin})
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await super().turn_on()
        await self.robot_brain.send(
            f'{self.name}_front.on();'
            f'{self.name}_back.on();'
        )

    async def turn_off(self) -> None:
        await super().turn_off()
        await self.robot_brain.send(
            f'{self.name}_front.off();'
            f'{self.name}_back.off();'
        )


class FlashlightPWMSimulation(FlashlightPWM, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'flashlight') -> None:
        self.name = name
        super().__init__()

    async def turn_on(self) -> None:
        if not await super().turn_on():
            return

    async def turn_off(self) -> None:
        await super().turn_off()
