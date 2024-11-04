import abc

import rosys
from rosys.helpers import remove_indentation


class Flashlight(rosys.hardware.Module, abc.ABC):
    MAX_HOT_DURATION = 10.0
    MAX_HOT_COUNT = 10

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.is_active: bool = False
        self.hot_time: float = 0
        self.hot_duration: float = 0

    async def activate(self, duration: float) -> None:
        async with self:
            await rosys.sleep(duration)

    async def deactivate(self) -> None:
        await self._deactivation()

    async def __aenter__(self):
        await self._activation()
        await rosys.sleep(1)

    async def __aexit__(self, exc_t, exc_v, exc_tb):
        await self._deactivation()

    async def _activation(self) -> None:
        if self.is_active:
            raise Exception('flashlight was already "on"')
        await self.turn_on()
        if rosys.time() - 10 > self.hot_time:  # reset hot_duration, hot lamp is some time ago
            self.hot_duration = 0
        self.hot_time = rosys.time()
        self.is_active = True

    async def _deactivation(self) -> None:
        if not self.is_active:
            return
        await self.turn_off()
        self.is_active = False
        self.hot_duration += rosys.time() - self.hot_time
        self.hot_time = rosys.time()  # remember last time the lamp was hot
        if self.hot_duration > 300:
            await self.cooldown()

    async def cooldown(self) -> None:
        rosys.notify('die Lampe wird für 10 Sek abgekühlt', type='info')
        await rosys.sleep(10)
        self.hot_duration = 0

    @abc.abstractmethod
    async def turn_on(self) -> None:
        self.log.info('turning on flashlight')
        pass

    @abc.abstractmethod
    async def turn_off(self) -> None:
        self.log.info('turning off flashlight')
        pass


class FlashlightHardware(Flashlight, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None,
                 name: str = 'flashlight',
                 pin: int = 5) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name}.Output({pin})
            {name}.on()
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await super().turn_on()
        await self.robot_brain.send(f'{self.name}.off()')

    async def turn_off(self) -> None:
        await super().turn_off()
        await self.robot_brain.send(f'{self.name}.on()')


class FlashlightSimulation(Flashlight, rosys.hardware.ModuleSimulation):

    def __init__(self, *,
                 name: str = 'flashlight') -> None:
        self.name = name
        super().__init__()

    async def turn_on(self) -> None:
        if not await super().turn_on():
            return

    async def turn_off(self) -> None:
        await super().turn_off()
