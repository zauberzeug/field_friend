import abc
from typing import Optional

import rosys
from rosys.hardware import Module, ModuleHardware, ModuleSimulation, RobotBrain

from .expander import ExpanderHardware


class ZAxis(Module):
    '''The z axis module is a simple example for a representation of real or simulated robot hardware.
    '''
    Z_AXIS_MAX_SPEED: float = 80_000
    MIN_Z: float = -0.197
    MAX_Z: float = -0.003
    STEPS_PER_MM: float = 1600

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.zaxis_end_t: bool = False
        self.zaxis_end_b: bool = False
        self.zaxis_position: int = 0
        self.zaxis_alarm: bool = False
        self.zaxis_idle: bool = False
        self.zaxis_is_referenced: bool = False
        self.zaxis_home_position: int = 0
        self.zaxis_drill_depth: float = self.MIN_Z
        self.end_stops_active: bool = True

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, world_position: float, speed: float) -> float:
        if not self.zaxis_is_referenced:
            rosys.notify('zaxis is not referenced, reference first')
            return None
        if self.zaxis_end_b or self.zaxis_end_t:
            rosys.notify('zaxis is in end stops, remove to move')
            return None
        if speed > self.Z_AXIS_MAX_SPEED:
            rosys.notify('zaxis speed is too high')
            return None
        if world_position < self.MIN_Z or world_position > self.MAX_Z:
            rosys.notify('zaxis position is out of range')
            return None
        steps = self.depth_to_steps(world_position)
        target_position = self.zaxis_home_position + steps
        return target_position

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def depth_to_steps(self, depth: float) -> int:
        steps = int((depth * 1000) * self.STEPS_PER_MM)
        return steps

    def steps_to_depth(self, steps: int) -> float:
        depth = (steps * (1 / self.STEPS_PER_MM)) / 1000
        return depth

    async def enable_end_stops(self, value: bool) -> None:
        self.end_stops_active = value
        self.log.info(f'end stops active = {value}')


class ZAxisHardware(ZAxis, ModuleHardware):
    '''The z axis module is a simple example for a representation of real or simulated robot hardware.
    '''
    CORE_MESSAGE_FIELDS: list[str] = ['z_end_t.level', 'z_end_b.level', 'zaxis.idle', 'zaxis.position', 'z_alarm.level']

    def __init__(self, robot_brain: RobotBrain, *,
                 name: str = 'zaxis',
                 expander: ExpanderHardware,
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 33,
                 end_t_pin: int = 13,
                 end_b_pin: int = 15,
                 ) -> None:
        self.name = name
        lizard_code = f'''
            {name} = {expander.name}.StepperMotor({step_pin}, {dir_pin}, 1, 1, 1, 1)
            z_alarm = {expander.name}.Input({alarm_pin})
            z_end_t = {expander.name}.Input({end_t_pin})
            z_end_b = {expander.name}.Input({end_b_pin})
            bool z_is_referencing = false;
            bool z_is_referenced = false;
            when zend_stops_active and z_is_referencing and z_end_t.level == 0 then
                {name}.stop();
                zend_stops_active = false;
            end
            when !zend_stops_active and z_is_referencing and z_end_t.level == 1 then 
                {name}.stop(); 
                zend_stops_active = true;
            end
            when !z_is_referencing and zend_stops_active and z_end_t.level == 0 then {name}.stop(); end
            when zend_stops_active and z_end_b.level == 0 then {name}.stop(); end
        '''
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, world_position: float, speed: float = 16_000) -> None:
        target_position = await super().move_to(world_position=world_position, speed=speed)
        if target_position is None:
            return
        await self.robot_brain.send(f'{self.name}.position({target_position}, {speed}, 160000);')
        if not await self.check_idle_or_alarm():
            return
        self.log.info(f'zaxis moved to {world_position}')

    async def check_idle_or_alarm(self) -> bool:
        while not self.zaxis_idle and not self.zaxis_alarm:
            await rosys.sleep(0.2)
        if self.zaxis_alarm:
            self.log.info("zaxis alarm")
            return False
        return True

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            if self.zaxis_end_t or self.zaxis_end_b:
                self.log.info('zaxis is in end stops, remove to reference')
                return False
            await self.robot_brain.send(
                'z_is_referencing = true;'
                f'{self.name}.speed({self.Z_AXIS_MAX_SPEED/2});'
            )
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed(-{self.Z_AXIS_MAX_SPEED/2});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed({self.Z_AXIS_MAX_SPEED/10});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send(f'{self.name}.speed(-{self.Z_AXIS_MAX_SPEED/10});')
            if not await self.check_idle_or_alarm():
                return False
            await self.robot_brain.send('z_is_referencing = false;')
            await rosys.sleep(0.1)
            self.zaxis_home_position = self.zaxis_position
            self.zaxis_is_referenced = True
            await self.move_to(self.MAX_Z, speed=self.Z_AXIS_MAX_SPEED/2)
            self.log.info('zaxis referenced')
            return True
        finally:
            await self.stop()

    async def enable_end_stops(self, value: bool) -> None:
        await super().enable_end_stops(value)
        await self.robot_brain.send(f'zend_stops_active = {str(value).lower()};')

    async def handle_core_output(self, time: float, words: list[str]) -> list[str]:
        self.zaxis_end_t = int(words.pop(0)) == 0
        self.zaxis_end_b = int(words.pop(0)) == 0
        self.zaxis_idle = words.pop(0) == 'true'
        self.zaxis_position = int(words.pop(0))
        self.zaxis_alarm = int(words.pop(0)) == 0
        if self.zaxis_alarm:
            self.zaxis_is_referenced = False
        return words


class ZAxisSimulation(ZAxis, ModuleSimulation):
    '''The z axis module is a simple example for a representation of real or simulated robot hardware.
    '''

    def __init__(self) -> None:
        super().__init__()
        self.zaxis_velocity: float = 0.0
        self.zaxis_target: Optional[float] = None

    async def stop(self) -> None:
        await super().stop()
        self.zaxis_velocity = 0.0
        self.zaxis_target = None

    async def move_to(self, world_position: float, speed: float = 16_000) -> None:
        target_position = await super().move_to(world_position=world_position, speed=speed)
        if target_position is None:
            return
        self.zaxis_target = target_position
        if self.zaxis_target > self.zaxis_position:
            self.zaxis_velocity = speed
        if self.zaxis_target < self.zaxis_position:
            self.zaxis_velocity = -speed
        while self.zaxis_target is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.zaxis_position = 0
        self.zaxis_home_position = 0
        self.zaxis_is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        return await super().step(dt)
