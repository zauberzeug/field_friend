import abc

from rosys.event import Event
from rosys.hardware import Module, ModuleHardware, ModuleSimulation
from rosys.hardware.robot_brain import RobotBrain


class EStop(Module, abc.ABC):
    '''The estop module is a simple example for a representation of real or simulated robot hardware.
    '''

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.ESTOP_TRIGGERED = Event()
        '''estop was triggered (argument: None)'''

        self.emergency_stop: bool = False


class EStopHardware(EStop, ModuleHardware):
    '''This module implements estop hardware.
    '''
    CORE_MESSAGE_FIELDS: list[str] = ['estop1.level', 'estop2.level']

    def __init__(self, robot_brain: RobotBrain, *,
                 name1: str = 'estop1',
                 pin1: int = 34,
                 name2: str = 'estop2',
                 pin2: int = 35) -> None:
        self.name1 = name1
        self.name2 = name2
        lizard_code = f'''
            {name1} = Input({pin1})
            {name2} = Input({pin2})
        '''
        core_message_fields: list[str] = [f'{self.name1}.level', f'{self.name2}.level']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    def handle_core_output(self, time: float, words: list[str]) -> None:

        estop1 = int(words.pop(0)) == 0
        estop2 = int(words.pop(0)) == 0
        self.emergency_stop = estop1 or estop2
        if self.emergency_stop:
            self.ESTOP_TRIGGERED.emit()


class EStopSimulation(EStop, ModuleSimulation):
    '''This module implements estop simulation.
    '''

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    async def step(self, dt: float) -> None:
        pass
