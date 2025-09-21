import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config.configuration import MowerConfiguration


class Mower(abc.ABC):
    def __init__(self, config: MowerConfiguration) -> None:
        super().__init__()
        self.config = config
        self.name = config.name

    @abc.abstractmethod
    async def turn_on(self) -> None:
        pass

    @abc.abstractmethod
    async def turn_off(self) -> None:
        pass

    async def stop(self) -> None:
        await self.turn_off()

    def developer_ui(self):
        ui.label('Mower:').classes('text-center text-bold')
        with ui.row():
            ui.button('Mower ON', on_click=self.turn_on)
            ui.button('Mower OFF', on_click=self.turn_off)


class MowerHardware(Mower, rosys.hardware.ModuleHardware):
    """This module implements extrernal mower hardware.

    on and off commands are forwarded to a given Robot Brain.
    """

    def __init__(self, config: MowerConfiguration, robot_brain: rosys.hardware.RobotBrain, *, expander: rosys.hardware.ExpanderHardware | None) -> None:
        Mower.__init__(self, config)
        self.pwm_name = f'{config.name}_pwm'
        self.enable_name = f'{config.name}_enable'
        lizard_code = remove_indentation(f'''
            {self.pwm_name} = {expander.name + "." if expander and config.pwm_on_expander else ""}PwmOutput({config.pwm_pin})
            {self.pwm_name}.duty = 0
            {self.pwm_name}.off()
            {self.enable_name} = {expander.name + "." if expander and config.enable_on_expander else ""}Output({config.enable_pin})
            {self.enable_name}.off()
        ''')
        rosys.hardware.ModuleHardware.__init__(self, robot_brain=robot_brain,
                                               lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await self.robot_brain.send(f'{self.pwm_name}.on()')
        await self.robot_brain.send(f'{self.enable_name}.on()')

    async def turn_off(self) -> None:
        await self.robot_brain.send(f'{self.enable_name}.off()')
        await self.robot_brain.send(f'{self.pwm_name}.off()')

    async def set_duty_cycle(self, duty_cycle: int) -> None:
        await self.robot_brain.send(f'{self.pwm_name}.duty={duty_cycle}')

    def developer_ui(self):
        super().developer_ui()
        ui.label('Duty Cycle:')
        ui.slider(value=0, min=0, max=255, on_change=lambda e: self.set_duty_cycle(int(e.value)))


class MowerSimulation(Mower, rosys.hardware.ModuleSimulation):
    async def turn_on(self) -> None:
        pass

    async def turn_off(self) -> None:
        pass
