import abc
from typing import Optional

import rosys
from rosys.hardware import Module, ModuleHardware, ModuleSimulation, RobotBrain

from .expander import ExpanderHardware


class YAxis(Module):
    '''The y axis module is a simple example for a representation of real or simulated robot hardware.
    '''
    Y_AXIS_MAX_SPEED: float = 80_000
    MIN_Y: float = -0.12
    MAX_Y: float = 0.12
    AXIS_OFFSET_Y = 0.123
    STEPS_PER_MM: float = 666.67

    AXIS_OFFSET_X = 0.2915

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.yaxis_end_l: bool = False
        self.yaxis_end_r: bool = False
        self.yaxis_position: int = 0
        self.yaxis_alarm: bool = False
        self.yaxis_idle: bool = False
        self.yaxis_is_referenced: bool = False
        self.yaxis_home_position: int = 0
        self.end_stops_active: bool = True

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, world_position: float, speed: float) -> float:
        if not self.yaxis_is_referenced:
            rosys.notify('yaxis is not referenced, reference first')
            return None
        if self.yaxis_end_b or self.yaxis_end_t:
            rosys.notify('yaxis is in end stops, remove to move')
            return None
        if speed > self.Y_AXIS_MAX_SPEED:
            rosys.notify('yaxis speed is too high')
            return None
        if world_position < self.MIN_Y or world_position > self.MAX_Y:
            rosys.notify('yaxis position is out of range')
            return None
        steps = self.linear_to_steps(world_position-self.AXIS_OFFSET_Y)
        target_position = self.yaxis_home_position + steps
        return target_position

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def linear_to_steps(self, linear: float) -> int:
        steps = int((linear * 1000) * self.STEPS_PER_MM)
        return steps

    def steps_to_linear(self, steps: int) -> float:
        linear = (steps * (1 / self.STEPS_PER_MM)) / 1000
        return linear

    async def enable_end_stops(self, value: bool) -> None:
        self.end_stops_active = value
        self.log.info(f'end stops active = {value}')


class YAxisHardware(YAxis, ModuleHardware):
    '''The y axis hardware module is a simple example for a representation of real robot hardware.
    '''
    CORE_MESSAGE_FIELDS: list[str] = ['y_end_l.level', 'y_end_r.level', 'yaxis.idle', 'yaxis.position', 'yaxis.alarm']

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'yaxis',
                 expander: ExpanderHardware,
                 step_pin: int = 19,
                 dir_pin: int = 18,
                 alarm_pin: int = 35,
                 end_l_pin: int = 36,
                 end_r_pin: int = 13) -> None:
        self.name = name
        lizard_code = f'''
            {name} = {expander.name}.StepperMotor({step_pin}, {dir_pin})
            y_alarm = {expander.name}.Input({alarm_pin})
            y_end_l = Input({end_l_pin})
            y_end_r = Input({end_r_pin})
            bool y_is_referencing = false
            bool yend_stops_active = true
            when yend_stops_active and y_is_referencing and y_end_l.level == 0 then
                {name}.stop();
                yend_stops_active = false
            end
            when !yend_stops_active and y_is_referencing and y_end_l.level == 1 then
                {name}.stop();
                yend_stops_active = true
            end
            when !y_is_referencing and yend_stops_active and y_end_l.level == 0 then {name}.stop(); end
            when yend_stops_active and y_end_r.level == 0 then {name}.stop(); end
        '''
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, world_position: float, speed: float = 80_000) -> None:
        target_position = await super().move_to(world_position, speed)
        if target_position is None:
            return
        await self.robot_brain.send(f'{self.name}.position({target_position}, {speed}, 160000);')
        if not await self.check_idle_or_alarm():
            return
        self.log.info(f'yaxis moved to {world_position}')

    async def check_idle_or_alarm(self) -> bool:
        while not self.yaxis_idle and not self.yaxis_alarm:
            await rosys.sleep(0.2)
        if self.yaxis_alarm:
            self.log.info('yaxis alarm')
            return False
        return True

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            if self.yaxis_end_l or self.yaxis_end_r:
                self.log.info('yaxis is in end stops, remove to reference')
                return False
            await self.robot_brain.send(
                'y_is_referencing = true;'
                f'{self.name}.speed({self.Y_AXIS_MAX_SPEED/2});'
            )
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed(-{self.Y_AXIS_MAX_SPEED/2});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed({self.Y_AXIS_MAX_SPEED/10});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed(-{self.Y_AXIS_MAX_SPEED/10});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'y_is_referencing = false;')
            await rosys.sleep(0.1)
            self.yaxis_home_position = self.yaxis_position
            self.yaxis_is_referenced = True
            await self.move_to(self.MAX_Y, speed=self.Y_AXIS_MAX_SPEED/2)
            self.log.info('yaxis referenced')
            return True
        finally:
            await self.stop()

    async def enable_end_stops(self, value: bool) -> None:
        await super().enable_end_stops(value)
        await self.robot_brain.send(f'yend_stops_active = {str(value).lower()};')

    async def handle_core_output(self, time: float, words: list[str]) -> list[str]:
        self.yaxis_end_l = int(words.pop(0)) == 0
        self.yaxis_end_r = int(words.pop(0)) == 0
        self.yaxis_idle = words.pop(0) == 'true'
        self.yaxis_position = int(words.pop(0))
        self.yaxis_alarm = int(words.pop(0)) == 0
        if self.yaxis_alarm:
            self.yaxis_is_referenced = False
        return words


class YAxisSimulation(YAxis, ModuleSimulation):
    '''The y axis simulation module is a simple example for a representation of simulated robot hardware.
    '''

    def __init__(self) -> None:
        super().__init__()
        self.yaxis_velocity: float = 0.0
        self.yaxis_target: Optional[float] = None

    async def stop(self) -> None:
        await super().stop()
        self.yaxis_velocity = 0.0
        self.yaxis_target = None

    async def move_to(self, world_position: float, speed: float = 80_000) -> None:
        target_position = await super().move_to(world_position, speed)
        if target_position is None:
            return
        self.yaxis_target = target_position
        if self.yaxis_target > self.yaxis_position:
            self.yaxis_velocity = speed
        if self.yaxis_target < self.yaxis_position:
            self.yaxis_velocity = -speed
        while self.yaxis_target is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.yaxis_position = 0
        self.yaxis_home_position = 0
        self.yaxis_is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.yaxis_position += int(dt * self.yaxis_velocity)
        if self.yaxis_target is not None:
            if (self.yaxis_velocity > 0) == (self.yaxis_position > self.yaxis_target):
                self.yaxis_position = self.yaxis_target
                self.yaxis_target = None
                self.yaxis_velocity = 0
