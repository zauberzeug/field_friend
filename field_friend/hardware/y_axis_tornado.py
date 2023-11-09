import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class YAxisTornado(rosys.hardware.Module, abc.ABC):
    """The y axis module is a simple example for a representation of real or simulated robot hardware."""

    MAX_SPEED: int = 80_000
    MIN_POSITION: float = -0.12
    MAX_POSITION: float = 0.12
    AXIS_OFFSET = 0.123
    STEPS_PER_M: float = 666.67 * 1000

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

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
        if speed > self.MAX_SPEED:
            raise RuntimeError(f'yaxis speed is too high, max speed is {self.MAX_SPEED}')
        if not self.MIN_POSITION <= position <= self.MAX_POSITION:
            raise RuntimeError(
                f'target position {position} ist out of yaxis range {self.MIN_POSITION} to {self.MAX_POSITION}')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, position: float) -> int:
        """Compute the number of steps to move the y axis to the given position.   

        The position is given in meters.
        """
        return int((position + self.AXIS_OFFSET) * self.STEPS_PER_M)

    def compute_position(self, steps: int) -> float:
        return steps / self.STEPS_PER_M + self.AXIS_OFFSET

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)


class YAxisHardwareTornado(YAxisTornado, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'yaxis_tornado',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 step_pin: int = 19,
                 dir_pin: int = 18,
                 alarm_pin: int = 35,
                 end_r_pin: int = 13,
                 end_l_pin: int = 36,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name}_motor = {expander.name + "." if motor_on_expander == True else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander == True else ""}Input({alarm_pin})
            {name}_end_l = {expander.name + "." if end_stops_on_expander == True else ""}Input({end_l_pin})
            {name}_end_r = {expander.name + "." if end_stops_on_expander == True else ""}Input({end_r_pin})
            {name} = MotorAxis({name}_motor, {name}_end_l, {name}_end_r)
        ''')
        core_message_fields = [
            f'{name}_end_l.level',
            f'{name}_end_r.level',
            f'{name}_motor.idle',
            f'{name}_motor.position',
            f'{name}_alarm.level'
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int = YAxisTornado.MAX_SPEED) -> None:
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

            # if in end l stop, disable end stops and move out
            if self.end_l:
                await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/10});')
                while self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # if in end r stop, move out
            if self.end_r:
                await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/10});')
                while self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # move to end r stop
            await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/6});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move out of end r stop
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/6});')
            while self.end_r:
                await rosys.sleep(0.2)

            # move slowly to end r stop
            await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/10});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move slowly out of end r stop
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/10});')
            while self.end_r:
                await rosys.sleep(0.2)

            # save position
            await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.position = 0;')
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

    def __init__(self) -> None:
        super().__init__()

        self.speed: int = 0
        self.target_steps: Optional[int] = None

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, position: float, speed: int = 80_000) -> None:
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
        if self.target_steps is not None:
            if (self.speed > 0) == (self.steps > self.target_steps):
                self.steps = self.target_steps
                self.target_steps = None
                self.speed = 0
