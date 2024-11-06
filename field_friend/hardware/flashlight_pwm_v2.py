import rosys
from rosys.helpers import remove_indentation

from .flashlight_v2 import FlashlightSimulationV2, FlashlightV2


class FlashlightPWMV2(FlashlightV2):
    ...


class FlashlightPWMHardwareV2(FlashlightPWMV2, rosys.hardware.ModuleHardware):

    UPDATE_INTERVAL = 5.0

    def __init__(self, robot_brain: rosys.hardware.RobotBrain,
                 bms: rosys.hardware.Bms, *,
                 expander: rosys.hardware.ExpanderHardware | None,
                 name: str = 'flashlight',
                 front_pin: int = 12,
                 back_pin: int = 25,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.bms = bms
        self.duty_cycle: float = 1
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if expander else ""}PwmOutput({front_pin})
            {name}.duty = 255
            {name}_back = {expander.name + "." if expander else ""}PwmOutput({back_pin})
            {name}_back.duty = 255
            {name}.shadow({name}_back)
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await super().turn_on()
        await self.robot_brain.send(f'{self.name}.on()')

    async def turn_off(self) -> None:
        await super().turn_off()
        await self.robot_brain.send(f'{self.name}.off()')

    async def set_duty_cycle(self) -> None:
        # get a 8 bit value for the duty cycle (0-255) no negative values
        duty = int(self.duty_cycle * 255)
        await self.robot_brain.send(
            f'{self.name}.duty={duty};'
            f'{self.name}_back.duty={duty};'
        )


class FlashlightPWMSimulationV2(FlashlightPWMV2, FlashlightSimulationV2):

    def __init__(self) -> None:
        self.duty_cycle = 1
        super().__init__()

    async def set_duty_cycle(self) -> None:
        pass
