import abc

import rosys
from nicegui import ui
from rosys.helpers import remove_indentation

from ..config import SprayerConfiguration


class Sprayer(rosys.hardware.Module, abc.ABC):
    def __init__(self, *, spray_radius: float, **kwargs) -> None:
        super().__init__(**kwargs)
        self.spray_radius = spray_radius

    @abc.abstractmethod
    async def activate_pump(self) -> None:
        pass

    @abc.abstractmethod
    async def deactivate_pump(self) -> None:
        pass

    @abc.abstractmethod
    async def open_valve(self) -> None:
        pass

    @abc.abstractmethod
    async def close_valve(self) -> None:
        pass

    async def stop(self) -> None:
        await self.close_valve()
        await self.deactivate_pump()

    @property
    def is_referenced(self) -> bool:
        return True

    async def try_reference(self) -> bool:
        return True

    def developer_ui(self) -> None:
        ui.label('Sprayer')
        with ui.row():
            ui.label('Pump')
            ui.button('On', on_click=self.activate_pump)
            ui.button('Off', on_click=self.deactivate_pump)
        with ui.row():
            ui.label('Valve')
            ui.button('Open', on_click=self.open_valve)
            ui.button('Close', on_click=self.close_valve)


class SprayerHardware(Sprayer, rosys.hardware.ModuleHardware):
    def __init__(self, config: SprayerConfiguration, robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None,
                 **kwargs) -> None:
        self.config = config
        self.name = config.name
        self.robot_brain = robot_brain
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {self.name}_valve = {expander.name + "." if expander and config.valve_pin_expander else ""}Output({config.valve_pin})
            {self.name}_pump = {expander.name + "." if expander and config.pump_pin_expander else ""}Output({config.pump_pin})
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, spray_radius=config.spray_radius, **kwargs)

    async def activate_pump(self) -> None:
        await self.robot_brain.send(f'{self.name}_pump.on()')

    async def deactivate_pump(self) -> None:
        await self.robot_brain.send(f'{self.name}_pump.off()')

    async def open_valve(self) -> None:
        await self.robot_brain.send(f'{self.name}_valve.on()')

    async def close_valve(self) -> None:
        await self.robot_brain.send(f'{self.name}_valve.off()')


class SprayerSimulation(Sprayer, rosys.hardware.ModuleSimulation):
    def __init__(self, *, spray_radius: float = 0.15, **kwargs) -> None:
        super().__init__(spray_radius=spray_radius, **kwargs)

    async def activate_pump(self) -> None:
        self.log.debug('Activating pump')

    async def deactivate_pump(self) -> None:
        self.log.debug('Deactivating pump')

    async def open_valve(self) -> None:
        self.log.debug('Opening valve')

    async def close_valve(self) -> None:
        self.log.debug('Closing valve')
