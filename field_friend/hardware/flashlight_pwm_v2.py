import rosys
from rosys.helpers import remove_indentation

from ..config import FlashlightConfiguration
from .flashlight_v2 import FlashlightSimulationV2, FlashlightV2


class FlashlightPWMV2(FlashlightV2):
    ...


class FlashlightPWMHardwareV2(FlashlightPWMV2, rosys.hardware.ModuleHardware):

    UPDATE_INTERVAL = 5.0

    def __init__(self, config: FlashlightConfiguration, robot_brain: rosys.hardware.RobotBrain,
                 bms: rosys.hardware.Bms, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        self.bms = bms
        self.duty_cycle: float = 1
        lizard_code = remove_indentation(f'''
            {config.name} = {expander.name + "." if expander else ""}PwmOutput({config.front_pin})
            {config.name}.duty = 255
            {config.name}_back = {expander.name + "." if expander else ""}PwmOutput({config.back_pin})
            {config.name}_back.duty = 255
            {config.name}.shadow({config.name}_back)
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning on flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_on()
        await self.robot_brain.send(f'{self.config.name}.on()')

    async def turn_off(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Turning off flashlight failed. Robot Brain is not ready.')
            return
        await super().turn_off()
        await self.robot_brain.send(f'{self.config.name}.off()')

    async def set_duty_cycle(self) -> None:
        if not self.robot_brain.is_ready:
            self.log.error('Setting duty cycle failed. Robot Brain is not ready.')
            return
        # get a 8 bit value for the duty cycle (0-255) no negative values
        duty = int(self.duty_cycle * 255)
        await self.robot_brain.send(
            f'{self.config.name}.duty={duty};'
            f'{self.config.name}_back.duty={duty};'
        )


class FlashlightPWMSimulationV2(FlashlightPWMV2, FlashlightSimulationV2):

    def __init__(self) -> None:
        self.duty_cycle = 1
        super().__init__()

    async def set_duty_cycle(self) -> None:
        pass
