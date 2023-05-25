import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class ZAxis(rosys.hardware.Module, abc.ABC):
    MAX_SPEED: float = 60_000
    MIN_DEPTH: float = 0.00
    MAX_DEPTH: float = 0.197
    STEPS_PER_M: float = 1600 * 1000
    REF_OFFSET: int = 0.0001

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False
        self.is_ref_enabled: bool = False
        self.is_end_b_enabled: bool = False

        self.ref_t: bool = False
        self.end_b: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, depth: float, speed: int) -> float:
        if not self.is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')
        if speed > self.MAX_SPEED:
            raise RuntimeError('zaxis speed is too high')
        if not self.MIN_DEPTH - self.REF_OFFSET <= depth <= self.MAX_DEPTH:
            raise RuntimeError('zaxis depth is out of range')

    @abc.abstractmethod
    async def move(self, speed: int) -> None:
        pass

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, depth: float) -> int:
        """Compute the number of steps to move the z axis to the given depth.

        Depth is positive and steps are negative when moving down.
        """
        return int(-depth * self.STEPS_PER_M)

    def compute_depth(self, steps: int) -> float:
        """Compute the depth of the z axis from the given number of steps.

        Depth is positive and steps are negative when moving down.
        """
        return -steps / self.STEPS_PER_M

    @property
    def depth(self) -> float:
        return self.compute_depth(self.steps)

    async def enable_end_stop(self, value: bool) -> None:
        pass

    async def enable_ref_stop(self, value: bool) -> None:
        pass

    async def return_to_reference(self) -> bool:
        try:
            await self.move_to(0 - self.REF_OFFSET, speed=self.MAX_SPEED/2)
        except RuntimeError as e:
            rosys.notify(e, type='negative')


class ZAxisHardware(ZAxis, rosys.hardware.ModuleHardware):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'zaxis',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 33,
                 ref_t_pin: int = 13,
                 end_b_pin: int = 15,) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if expander else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if expander else ""}Input({alarm_pin})
            {name}_ref_t = {expander.name + "." if expander else ""}Input({ref_t_pin})
            {name}_end_b = {expander.name + "." if expander else ""}Input({end_b_pin})
            bool {name}_is_referencing = false;
            bool {name}_ref_t_enabled = true;
            bool {name}_end_b_enabled = true;
            when {name}_ref_t_enabled and {name}_is_referencing and {name}_ref_t.level == 0 then
                {name}.stop();
                {name}_ref_t_enabled = false;
            end
            when !{name}_ref_t_enabled and {name}_is_referencing and {name}_ref_t.level == 1 then 
                {name}.stop(); 
                {name}_ref_t_enabled = true;
            end
            when {name}_end_b_enabled and {name}_end_b.level == 0 then {name}.stop(); end
        ''')
        core_message_fields = [
            f'{name}_ref_t.level',
            f'{name}_end_b.level',
            f'{name}.idle',
            f'{name}.position',
            f'{name}_alarm.level',
            f'{name}_ref_t_enabled',
            f'{name}_end_b_enabled',
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, depth: float, speed: float = ZAxis.MAX_SPEED) -> None:
        try:
            await super().move_to(depth, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        steps = self.compute_steps(depth)

        await self.robot_brain.send(f'{self.name}.position({steps}, {speed}, 60000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            rosys.notify('z_axis fault detected', type='negative')
            return

    async def move(self, speed: int) -> None:
        try:
            await super().move(speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        await self.robot_brain.send(f'{self.name}.speed({speed}, 8000)')

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
            self.log.info('zaxis referencing...')

            await self.robot_brain.send(f'{self.name}_is_referencing = true;')
            await rosys.sleep(0.5)

            # if in end b stop, diasble end b stop
            if self.end_b:
                await self.enable_end_stop(False)
                await rosys.sleep(0.5)

            # if in ref t stop, move out
            if self.ref_t:
                await self.enable_ref_stop(False)
                await rosys.sleep(0.5)
                await self.robot_brain.send(
                    f'{self.name}.speed(-{self.MAX_SPEED/2});'
                )
                while self.ref_t:
                    await rosys.sleep(0.2)

            # move to top ref
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/2});')
            while not self.ref_t:
                await rosys.sleep(0.2)

            # move out of top ref
            await self.robot_brain.send(f'{self.name}.speed(-{self.MAX_SPEED/2});')
            while self.ref_t:
                await rosys.sleep(0.2)

            # move slowly to top ref
            await self.robot_brain.send(f'{self.name}.speed({self.MAX_SPEED/10});')
            while not self.ref_t:
                await rosys.sleep(0.2)

            # save position
            await rosys.sleep(0.2)
            await self.robot_brain.send(
                f'{self.name}_is_referencing = false;'
            )
            await rosys.sleep(0.5)
            await self.robot_brain.send(
                f'{self.name}.position = 0;'
            )
            await rosys.sleep(0.5)
            self.is_referenced = True

            # drive to reference with offset
            await self.return_to_reference()
            await self.enable_end_stop(True)

            self.log.info('zaxis referenced')
            return True
        except Exception as e:
            self.log.error(f'zaxis reference failed: {e}')
            return False
        finally:
            await self.stop()

    async def enable_ref_stop(self, value: bool) -> None:
        await super().enable_ref_stop(value)
        self.log.info(f'zaxis ref t stop enabled: {value}')
        await self.robot_brain.send(f'{self.name}_ref_t_enabled = {str(value).lower()};')

    async def enable_end_stop(self, value: bool) -> None:
        await super().enable_end_stop(value)
        self.log.info(f'zaxis end b stop enabled: {value}')
        await self.robot_brain.send(f'{self.name}_end_b_enabled = {str(value).lower()};')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.ref_t = int(words.pop(0)) == 0
        self.end_b = int(words.pop(0)) == 0
        if self.end_b:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False
        self.is_ref_enabled = words.pop(0) == 'true'
        self.is_end_b_enabled = words.pop(0) == 'true'


class ZAxisSimulation(ZAxis, rosys.hardware.ModuleSimulation):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self) -> None:
        super().__init__()
        self.speed: int = 0
        self.target_steps: Optional[int] = None
        self.reference_steps: int = 0

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, depth: float, speed: int = ZAxis.MAX_SPEED) -> None:
        try:
            await super().move_to(depth, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        self.target_steps = self.compute_steps(depth)
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def move(self, speed: int) -> None:
        await super().move(speed)
        self.speed = speed

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.steps = 0
        self.reference_steps = 0
        self.is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        self.steps += int(dt * self.speed)
        self.is_reference_active = self.steps >= self.reference_steps
        self.idle = self.speed == 0
        if self.target_steps is not None and (self.speed > 0) == (self.steps > self.target_steps):
            self.steps = self.target_steps
            self.target_steps = None
            self.speed = 0
