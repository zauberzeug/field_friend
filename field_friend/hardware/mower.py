import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config.configuration import MowerConfiguration


class Mower(abc.ABC):
    def __init__(self, config: MowerConfiguration) -> None:
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
    START_DUTY_CYCLE = 25
    TARGET_DUTY_CYCLE = 165
    MAX_DUTY_CYCLE = 170

    def __init__(self, config: MowerConfiguration, robot_brain: rosys.hardware.RobotBrain, *, expander: rosys.hardware.ExpanderHardware | None) -> None:
        Mower.__init__(self, config)
        self.pwm_name = f'{config.name}_pwm'
        self.enable_name = f'{config.name}_enable'
        lizard_code = remove_indentation(f'''
            {self.pwm_name} = {expander.name + "." if expander and config.pwm_on_expander else ""}PwmOutput({config.pwm_pin})
            {self.pwm_name}.duty = 0
            {self.pwm_name}.frequency = 1000
            {self.enable_name} = {expander.name + "." if expander and config.enable_on_expander else ""}Output({config.enable_pin})
        ''')
        self._is_running = False
        self._target_duty_cycle = self.TARGET_DUTY_CYCLE
        rosys.hardware.ModuleHardware.__init__(self, robot_brain, lizard_code)

    async def turn_on(self) -> None:
        await self.set_duty_cycle(12)
        await self.robot_brain.send(f'{self.pwm_name}.on()')
        await self.robot_brain.send(f'{self.enable_name}.on()')
        await rosys.sleep(1)
        await self.set_duty_cycle(self._target_duty_cycle)
        self._is_running = True

    async def turn_off(self) -> None:
        await self.robot_brain.send(f'{self.enable_name}.off()')
        await self.robot_brain.send(f'{self.pwm_name}.off()')
        self._is_running = False

    async def toggle(self) -> None:
        if self._is_running:
            await self.turn_off()
        else:
            await self.turn_on()

    async def set_duty_cycle(self, duty_cycle: int) -> None:
        self._target_duty_cycle = duty_cycle
        await self.robot_brain.send(f'{self.pwm_name}.duty={duty_cycle}')

    async def set_frequency(self, frequency: int) -> None:
        await self.robot_brain.send(f'{self.pwm_name}.frequency={frequency}')

    def developer_ui(self):
        super().developer_ui()
        with ui.row():
            async def set_duty_cycle(e):
                await self.set_duty_cycle(int(e.value))
            slider = ui.slider(min=self.START_DUTY_CYCLE, max=self.MAX_DUTY_CYCLE,
                               value=self._target_duty_cycle, on_change=set_duty_cycle)
            ui.label().bind_text_from(slider, 'value', backward=lambda value: f'Duty Cycle: {value}')


class MowerSimulation(Mower, rosys.hardware.ModuleSimulation):
    async def turn_on(self) -> None:
        pass

    async def turn_off(self) -> None:
        pass
