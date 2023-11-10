import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class YAxisTornado(rosys.hardware.Module, abc.ABC):
    """The y axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, max_speed, min_position, max_position, axis_offset, steps_per_m, **kwargs) -> None:
        super().__init__(**kwargs)

        self.max_speed: int = max_speed
        self.min_position: float = min_position
        self.max_position: float = max_position
        self.axis_offset: float = axis_offset
        self.steps_per_m: float = steps_per_m

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False

        self.end_l: bool = False
        self.end_r: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, position: float, speed: int) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if speed > self.max_speed:
            raise RuntimeError(f'yaxis speed is too high, max speed is {self.max_speed}')
        if not self.min_position <= position <= self.max_position:
            raise RuntimeError(
                f'target position {position} ist out of yaxis range {self.min_position} to {self.max_position}')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, position: float) -> int:
        """Compute the number of steps to move the y axis to the given position.   

        The position is given in meters.
        """
        return int((position + self.axis_offset) * self.steps_per_m)

    def compute_position(self, steps: int) -> float:
        return steps / self.steps_per_m - self.axis_offset

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)


class YAxisHardwareTornado(YAxisTornado, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'yaxis_tornado',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 max_speed: int = 80_000,
                 min_position: float = -0.12,
                 max_position: float = 0.12,
                 axis_offset: float = 0.123,
                 steps_per_m: float = 666.67 * 1000,
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 36,
                 end_r_pin: int = 19,
                 end_l_pin: int = 21,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name}_motor = {expander.name + "." if motor_on_expander and expander else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander and expander else ""}Input({alarm_pin})
            {name}_end_l = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_l_pin})
            {name}_end_r = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_r_pin})
            {name} = {expander.name + "." if motor_on_expander and expander else ""}MotorAxis({name}_motor, {name}_end_r, {name}_end_l)
        ''')
        core_message_fields = [
            f'{name}_end_l.level',
            f'{name}_end_r.level',
            f'{name}_motor.idle',
            f'{name}_motor.position',
            f'{name}_alarm.level'
        ]
        super().__init__(
            max_speed=max_speed,
            min_position=min_position,
            max_position=max_position,
            axis_offset=axis_offset,
            steps_per_m=steps_per_m,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int = 0) -> None:
        if speed == 0:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.info(f'could not move yaxis to {position} because of {error}')
            raise Exception(f'could not move yaxis to {position} because of {error}')
        steps = self.compute_steps(position)
        await self.robot_brain.send(f'{self.name}.position({steps}, {speed}, 250000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception

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

            # if in end l stop, move out
            if self.end_l:
                self.log.info('already in end_l moving out of end_l stop')
                await self.robot_brain.send(f'{self.name}.speed(-{self.max_speed/10});')
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # if in end r stop, move out
            if self.end_r:
                self.log.info('already in end_r moving out of end_r stop')
                await self.robot_brain.send(f'{self.name}.speed({self.max_speed/10});')
                while self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # move to end r stop
            self.log.info('moving to end_r stop')
            await self.robot_brain.send(f'{self.name}.speed(-{self.max_speed/6});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move out of end r stop
            self.log.info('moving out of end_r stop')
            await self.robot_brain.send(f'{self.name}.speed({self.max_speed/6});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            await rosys.sleep(1)
            # move slowly to end r stop
            self.log.info('moving slowly to end_r stop')
            await self.robot_brain.send(f'{self.name}.speed(-{self.max_speed/10});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move slowly out of end r stop
            self.log.info('moving slowly out of end_r stop')
            await self.robot_brain.send(f'{self.name}.speed({self.max_speed/10});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # save position
            await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}_motor.position = 0;')
            await rosys.sleep(0.5)
            self.is_referenced = True
            return True
        except Exception as error:
            self.log.error(f'could not reference yaxis because of {error}')
            return False
        finally:
            await self.stop()

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


class YAxisSimulationTornado(YAxisTornado, rosys.hardware.ModuleSimulation):
    '''The y axis simulation module is a simple example for a representation of simulated robot hardware.
    '''

    def __init__(self,
                 max_speed: int = 80_000,
                 min_position: float = -0.12,
                 max_position: float = 0.12,
                 axis_offset: float = 0.123,
                 steps_per_m: float = 666.67 * 1000,
                 ) -> None:
        self.speed: int = 0
        self.target_steps: Optional[int] = None
        super().__init__(
            max_speed=max_speed,
            min_position=min_position,
            max_position=max_position,
            axis_offset=axis_offset,
            steps_per_m=steps_per_m,
        )

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, position: float, speed: int = 0) -> None:
        if speed == 0:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        self.target_steps = self.compute_steps(position)
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.steps = 0
        self.is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.steps += int(dt * self.speed)
        self.idle = self.speed == 0
        if self.target_steps is not None:
            if (self.speed > 0) == (self.steps > self.target_steps):
                self.steps = self.target_steps
                self.target_steps = None
                self.speed = 0
