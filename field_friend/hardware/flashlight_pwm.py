from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .flashlight_v2 import FlashlightV2


class FlashlightPWM(FlashlightV2):
    ...


class FlashlightPWMHardware(FlashlightPWM, rosys.hardware.ModuleHardware):

    UPDATE_INTERVAL = 5.0

    def __init__(self, robot_brain: rosys.hardware.RobotBrain,
                 bms: rosys.hardware.Bms, *,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 name: str = 'flashlight',
                 pin: int = 12,
                 rated_voltage: float = 23.0,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.bms = bms
        self.rated_voltage = rated_voltage
        self.last_update: float = 0
        self.duty_cycle: float = 0.1
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if expander else ""}PwmOutput({pin})
            {name}.duty = 20
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        rosys.on_repeat(self._set_duty_cycle, 1.0)

    async def turn_on(self) -> None:
        await super().turn_on()
        await self.robot_brain.send(
            f'{self.name}.on();'
        )

    async def turn_off(self) -> None:
        await super().turn_off()
        await self.robot_brain.send(
            f'{self.name}.off();'
        )

    async def _set_duty_cycle(self) -> None:
        if rosys.time() > self.last_update + self.UPDATE_INTERVAL:

            self.last_update = rosys.time()
            current_voltage = self.bms.state.voltage
            if current_voltage is None:
                return
            self.duty_cycle = (self.rated_voltage / current_voltage)**2
            if self.duty_cycle > 1:
                self.duty_cycle = 1
            # get a 8 bit value for the duty cycle (0-255) no negative values
            duty = int(self.duty_cycle * 255)

            await self.robot_brain.send(
                f'{self.name}.duty={duty};'
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
