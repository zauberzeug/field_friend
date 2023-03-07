import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class ZAxis(rosys.hardware.Module, abc.ABC):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    Z_AXIS_MAX_SPEED: float = 80_000
    MIN_Z: float = -0.197
    MAX_Z: float = -0.003
    STEPS_PER_MM: float = 1600

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.end_t: bool = False
        self.end_b: bool = False
        self.position: int = 0
        self.alarm: bool = False
        self.idle: bool = False
        self.is_referenced: bool = False
        self.home_position: int = 0
        self.drill_depth: float = self.MIN_Z
        self.end_stops_active: bool = True

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, world_position: float, speed: float) -> float:
        if not self.is_referenced:
            rosys.notify('zaxis is not referenced, reference first')
            return None
        if self.end_b or self.end_t:
            rosys.notify('zaxis is in end stops, remove to move')
            return None
        if speed > self.Z_AXIS_MAX_SPEED:
            rosys.notify('zaxis speed is too high')
            return None
        if world_position < self.MIN_Z or world_position > self.MAX_Z:
            rosys.notify('zaxis position is out of range')
            return None
        steps = self.depth_to_steps(world_position)
        target_position = self.home_position + steps
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


class ZAxisHardware(ZAxis, rosys.hardware.ModuleHardware):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'zaxis',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 33,
                 end_t_pin: int = 13,
                 end_b_pin: int = 15,) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name}.StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name}.Input({alarm_pin})
            {name}_end_t = {expander.name}.Input({end_t_pin})
            {name}_end_b = {expander.name}.Input({end_b_pin})
            bool {name}_is_referencing = false;
            bool {name}_end_stops_active = true
            when {name}_end_stops_active and {name}_is_referencing and {name}_end_t.level == 0 then
                {name}.stop();
                {name}_end_stops_active = false;
            end
            when !{name}_end_stops_active and {name}_is_referencing and {name}_end_t.level == 1 then 
                {name}.stop(); 
                {name}_end_stops_active = true;
            end
            when !{name}_is_referencing and {name}_end_stops_active and {name}_end_t.level == 0 then {name}.stop(); end
            when {name}_end_stops_active and {name}_end_b.level == 0 then {name}.stop(); end
        ''')
        core_message_fields = [f'{name}_end_t.level', f'{name}_end_b.level',
                               f'{name}.idle', f'{name}.position', f'{name}_alarm.level']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, world_position: float, speed: float = 80_000) -> None:
        target_position = await super().move_to(world_position=world_position, speed=speed)
        if target_position is None:
            return
        await self.robot_brain.send(f'{self.name}.position({target_position}, {speed}, 160000);')
        if not await self.check_idle_or_alarm():
            return
        self.log.info(f'zaxis moved to {world_position}')

    async def check_idle_or_alarm(self) -> bool:
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.info("zaxis alarm")
            return False
        return True

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            if not self.end_stops_active:
                self.log.warning('end stops not activated')
                return False
            if self.end_t or self.end_b:
                self.log.info('zaxis is in end stops, remove to reference')
                return False
            await self.robot_brain.send(
                f'{self.name}_is_referencing = true;'
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
            await self.robot_brain.send(f'{self.name}_is_referencing = false;')
            await rosys.sleep(0.1)
            self.home_position = self.position
            self.is_referenced = True
            await self.move_to(self.MAX_Z, speed=self.Z_AXIS_MAX_SPEED/2)
            self.log.info('zaxis referenced')
            return True
        finally:
            await self.stop()

    async def enable_end_stops(self, value: bool) -> None:
        await super().enable_end_stops(value)
        await self.robot_brain.send(f'{self.name}_end_stops_active = {str(value).lower()};')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_t = int(words.pop(0)) == 0
        self.end_b = int(words.pop(0)) == 0
        self.idle = words.pop(0) == 'true'
        self.position = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False


class ZAxisSimulation(ZAxis, rosys.hardware.ModuleSimulation):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self) -> None:
        super().__init__()
        self.velocity: float = 0.0
        self.target: Optional[float] = None

    async def stop(self) -> None:
        await super().stop()
        self.velocity = 0.0
        self.target = None

    async def move_to(self, world_position: float, speed: float = 16_000) -> None:
        target_position = await super().move_to(world_position=world_position, speed=speed)
        if target_position is None:
            return
        self.target = target_position
        if self.target > self.position:
            self.velocity = speed
        if self.target < self.position:
            self.velocity = -speed
        while self.target is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.position = 0
        self.home_position = 0
        self.is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.position += int(dt * self.velocity)
        if self.target is not None:
            if (self.velocity > 0) == (self.position > self.target):
                self.position = self.target
                self.target = None
                self.velocity = 0
