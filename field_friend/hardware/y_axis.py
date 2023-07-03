import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class YAxis(rosys.hardware.Module, abc.ABC):
    """The y axis module is a simple example for a representation of real or simulated robot hardware."""

    MAX_SPEED: float = 80_000
    MIN_POSITION: float = -0.12
    MAX_POSITION: float = 0.12
    AXIS_OFFSET = 0.123
    STEPS_PER_M: float = 666.67 * 1000

    OFFSET_X = 0.2915

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False
        self.end_stops_enabled: bool = True
        self.homesteps: int = 0

        self.end_l: bool = False
        self.end_r: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, position: float, speed: float) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if speed > self.MAX_SPEED:
            raise RuntimeError(f'yaxis speed is too high, max speed is {self.MAX_SPEED}')
        if not self.MIN_POSITION <= position <= self.MAX_POSITION:
            raise RuntimeError(f'target position ist out of yaxis range')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, position: float) -> int:
        """Compute the number of steps to move the y axis to the given position.   

        The position is given in meters.
        """
        return int((-position + self.AXIS_OFFSET) * self.STEPS_PER_M + self.homesteps)

    def compute_position(self, steps: int) -> float:
        return -steps / self.STEPS_PER_M + self.AXIS_OFFSET

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)

    async def enable_end_stops(self, value: bool) -> None:
        self.end_stops_enabled = value
        self.log.info(f'end stops enabled = {value}')


class YAxisHardware(YAxis, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'yaxis',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 step_pin: int = 19,
                 dir_pin: int = 18,
                 alarm_pin: int = 35,
                 end_l_pin: int = 36,
                 end_r_pin: int = 13,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if motor_on_expander == True else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander == True else ""}Input({alarm_pin})
            {name}_end_l = {expander.name + "." if end_stops_on_expander == True else ""}Input({end_l_pin})
            {name}_end_r = {expander.name + "." if end_stops_on_expander == True else ""}Input({end_r_pin})
            bool {name}_is_referencing = false
            bool {name}_end_stops_enabled = true
            when {name}_end_stops_enabled and {name}_is_referencing and {name}_end_l.level == 0 then
                {name}.stop();
                {name}_end_stops_enabled = false
            end
            when !{name}_end_stops_enabled and {name}_is_referencing and {name}_end_l.level == 1 then
                {name}.stop();
                {name}_end_stops_enabled = true
            end
            when !{name}_is_referencing and {name}_end_stops_enabled and {name}_end_l.level == 0 then {name}.stop(); end
            when {name}_end_stops_enabled and {name}_end_r.level == 0 then {name}.stop(); end
        ''')
        core_message_fields = [
            f'{name}_end_l.level',
            f'{name}_end_r.level',
            f'{name}.idle',
            f'{name}.position',
            f'{name}_alarm.level'
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: float = YAxis.MAX_SPEED) -> None:
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            rosys.notify(error, type='negative')
            self.log.info(f'could not move yaxis to {position} because of {error}')
            raise RuntimeError(f'could not move yaxis to {position} because of {error}')
            return
        steps = self.compute_steps(position)
        await self.robot_brain.send(f'{self.name}.position({steps}, {speed}, 250000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            rosys.notify('yaxis fault detected', type='negative')
            return

    async def check_idle_or_alarm(self) -> bool:
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.info('yaxis alarm')
            return False
        return True

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            if not self.end_stops_enabled:
                self.log.warning('end stops not enabled')
                return False

            # if in end r stop, disable end stops and move out
            if self.end_r:
                await self.enable_end_stops(False)
                await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/10});')
                while self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')
                await self.enable_end_stops(True)

            # if in end l stop, move out
            if self.end_l:
                await self.enable_end_stops(False)
                await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/10});')
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')
                await self.enable_end_stops(True)

            await self.robot_brain.send(f'{self.name}_is_referencing = true;')

            # move to end l stop
            await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/6});')
            while not self.end_l:
                await rosys.sleep(0.2)

            # move out of end l stop
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/6});')
            while self.end_l:
                await rosys.sleep(0.2)

            # move slowly to end l stop
            await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/10});')
            while not self.end_l:
                await rosys.sleep(0.2)

            # move slowly out of end l stop
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/10});')
            while self.end_l:
                await rosys.sleep(0.2)

            # save position
            await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}_is_referencing = false;')
            await self.robot_brain.send(f'{self.name}.position = 0;')  # ToDO: This seems to be buggy on expander
            await rosys.sleep(0.5)
            self.is_referenced = True
            self.homesteps = self.steps
            self.log.info(f'yaxis referenced, homesteps: {self.homesteps}')
            return True
        except Exception as error:
            self.log.error(f'could not reference yaxis because of {error}')
            return False
        finally:
            await self.stop()

    async def enable_end_stops(self, value: bool) -> None:
        await super().enable_end_stops(value)
        await self.robot_brain.send(f'{self.name}_end_stops_enabled = {str(value).lower()};')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_l = int(words.pop(0)) == 0
        self.end_r = int(words.pop(0)) == 0
        if self.end_l or self.end_r:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False


class YAxisSimulation(YAxis, rosys.hardware.ModuleSimulation):
    '''The y axis simulation module is a simple example for a representation of simulated robot hardware.
    '''

    def __init__(self) -> None:
        super().__init__()

        self.speed: int = 0
        self.target_steps: Optional[float] = None

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, world_position: float, speed: float = 80_000) -> None:
        try:
            await super().move_to(world_position, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        self.target_steps = self.compute_steps(world_position)
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.steps = 0
        self.homesteps = 0
        self.is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.steps += int(dt * self.speed)
        if self.target_steps is not None:
            if (self.speed > 0) == (self.steps > self.target_steps):
                self.steps = self.target_steps
                self.target_steps = None
                self.speed = 0
