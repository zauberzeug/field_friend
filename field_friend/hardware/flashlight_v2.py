import abc

import rosys
from rosys.helpers import remove_indentation

from ..config import FlashlightConfiguration


class FlashlightV2(rosys.hardware.Module, abc.ABC):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.is_active: bool = False

    async def __aenter__(self):
        await self.turn_on()

    async def __aexit__(self, exc_t, exc_v, exc_tb):
        await self.turn_off()

    @abc.abstractmethod
    async def turn_on(self) -> None:
        self.is_active = True

    @abc.abstractmethod
    async def turn_off(self) -> None:
        self.is_active = False


class FlashlightHardwareV2(FlashlightV2, rosys.hardware.ModuleHardware):

    def __init__(self, config: FlashlightConfiguration, robot_brain: rosys.hardware.RobotBrain,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        # TODO: is this always on the expander? otherwise it will break -> config mit Flashlight auf dem Expander? Sonst nicht optional
        lizard_code = remove_indentation(f'''
            {config.name}_front = {expander.name + "." if expander else ""}Output({config.front_pin})
            {config.name}_back = {expander.name + "." if expander else ""}Output({config.back_pin})
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await super().turn_on()
        await self.robot_brain.send(
            f'{self.config.name}_front.on();'
            f'{self.config.name}_back.on();'
        )

    async def turn_off(self) -> None:
        await super().turn_off()
        await self.robot_brain.send(
            f'{self.config.name}_front.off();'
            f'{self.config.name}_back.off();'
        )


class FlashlightSimulationV2(FlashlightV2, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'flashlight') -> None:
        self.name = name
        super().__init__()

    async def turn_on(self) -> None:
        await super().turn_on()

    async def turn_off(self) -> None:
        await super().turn_off()
