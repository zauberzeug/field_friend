import abc
from typing import Optional

import numpy as np
import rosys
from rosys.helpers import ramp, remove_indentation


class ChainAxis(rosys.hardware.Module, abc.ABC):
    # GEAR = 40
    # STEPS_PER_REV = 1600
    CIRCUMFERENCE = 51.85 * 2 * np.pi / 1000
    # STEPS_PER_M: float = STEPS_PER_REV * GEAR / CIRCUMFERENCE
    DEFAULT_SPEED: int = 20000  # TODO: make configurable (U2=40000, U3 = 20000)
    MIN_POSITION = -0.14235
    MAX_POSITION = 0.14235
    CHAIN_RADIUS = 0.05185
    RADIUS_STEPS = 8400  # TODO: make configurable (U2=16000, U3 = 8400)
    REF_OFFSET = 1500
    POSITION_OFFSET = CHAIN_RADIUS / RADIUS_STEPS * REF_OFFSET
    TOP_DOWN_FACTOR = 1.0589

    def __init__(self, min_position: float = -0.10, max_position: float = 0.10, **kwargs) -> None:
        super().__init__(**kwargs)
        self.min_position = min_position
        self.max_position = max_position

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False
        self.steps_to_end: int = -65000

        self.ref_l: bool = False
        self.ref_r: bool = False
        self.ref_t: bool = False

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    def compute_steps(self, position: float) -> int:
        if self.MIN_POSITION <= position <= self.MIN_POSITION + self.CHAIN_RADIUS:
            self.log.info(f'position => right side: {position}')
            steps = int(ramp(position, self.MIN_POSITION + self.POSITION_OFFSET, self.MIN_POSITION +
                        self.CHAIN_RADIUS, 0, -self.RADIUS_STEPS+self.REF_OFFSET, clip=True))
        elif self.MIN_POSITION + self.CHAIN_RADIUS <= position <= self.MAX_POSITION - self.CHAIN_RADIUS:
            self.log.info(f'position => middle: {position}')
            steps = int(ramp(position, self.MIN_POSITION + self.CHAIN_RADIUS,
                        self.MAX_POSITION - self.CHAIN_RADIUS, -self.RADIUS_STEPS+self.REF_OFFSET, self.steps_to_end + self.RADIUS_STEPS - self.REF_OFFSET, clip=True))
        else:
            self.log.info(f'position => left side: {position}')
            steps = int(ramp(position, self.MAX_POSITION - self.CHAIN_RADIUS, self.MAX_POSITION-self.POSITION_OFFSET,
                        self.steps_to_end + self.RADIUS_STEPS - self.REF_OFFSET, self.steps_to_end, clip=True))
        return steps

    def compute_position(self, steps: int) -> float:
        if steps > -self.RADIUS_STEPS + self.REF_OFFSET:
            position = ramp(steps, 0, -self.RADIUS_STEPS+self.REF_OFFSET, self.MIN_POSITION +
                            self.POSITION_OFFSET, self.MIN_POSITION + self.CHAIN_RADIUS)
        elif -self.RADIUS_STEPS + self.REF_OFFSET > steps >= self.steps_to_end + self.RADIUS_STEPS - self.REF_OFFSET:
            position = ramp(steps, -self.RADIUS_STEPS + self.REF_OFFSET, self.steps_to_end + self.RADIUS_STEPS - self.REF_OFFSET,
                            self.MIN_POSITION + self.CHAIN_RADIUS, self.MAX_POSITION - self.CHAIN_RADIUS)
        else:
            position = ramp(steps, self.steps_to_end + self.RADIUS_STEPS - self.REF_OFFSET, self.steps_to_end,
                            self.MAX_POSITION - self.CHAIN_RADIUS, self.MAX_POSITION - self.POSITION_OFFSET)
        return position

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)

    @abc.abstractmethod
    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.DEFAULT_SPEED
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.min_position <= position <= self.max_position:
            raise RuntimeError(f'target yaxis {position} is out of range')
        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, move to top reference first')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    async def return_to_reference(self) -> None:
        if self.position < 0:
            await self.return_to_r_ref()
        else:
            await self.return_to_l_ref()

    @abc.abstractmethod
    async def return_to_r_ref(self) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, move to top reference first')

    @abc.abstractmethod
    async def return_to_l_ref(self) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, move to top reference first')

    @abc.abstractmethod
    async def move_dw_to_l_ref(self) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, move to top reference first')

    @abc.abstractmethod
    async def move_dw_to_r_ref(self) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, move to top reference first')

    async def reset(self) -> bool:
        return True


class ChainAxisHardware(ChainAxis, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'chain_axis',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 min_position: float = -0.10,
                 max_position: float = 0.10,
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 13,
                 ref_t_pin: int = 21,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if motor_on_expander else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander else ""}Input({alarm_pin})
            {name}_ref_t = {expander.name + "." if end_stops_on_expander else ""}Input({ref_t_pin})

            bool {name}_ref_r_is_referencing = false
            bool {name}_ref_l_is_referencing = false
            bool {name}_ref_t_stop_enabled = false
            bool {name}_ref_r_return = false 
            bool {name}_ref_l_return = false
            bool {name}_resetting = false

            when {name}_ref_r_is_referencing and {name}_ref_t_stop_enabled and {name}_ref_t.level == 1 then
                {name}.stop();
            end

            when {name}_ref_r_is_referencing and !{name}_ref_t_stop_enabled and {name}_ref_t.level == 0 then
                {name}.stop();
            end

            when {name}_ref_l_is_referencing and {name}_ref_t_stop_enabled and {name}_ref_t.level == 1 then
                {name}.stop();
            end

            when {name}_ref_l_is_referencing and !{name}_ref_t_stop_enabled and {name}_ref_t.level == 0 then
                {name}.stop();
            end

            when {name}_ref_r_return and {name}_ref_t.level == 1 then
                {name}_ref_t_stop_enabled = true;
            end

            when {name}_ref_r_return and {name}_ref_t_stop_enabled and {name}_ref_t.level == 0 then
                {name}.stop();
                {name}_ref_r_return = false;
                {name}_ref_t_stop_enabled = false;
            end

            when {name}_ref_l_return and {name}_ref_t.level == 1 then
                {name}_ref_t_stop_enabled = true;
            end

            when {name}_ref_l_return and {name}_ref_t_stop_enabled and {name}_ref_t.level == 0 then
                {name}.stop();
                {name}_ref_l_return = false;
                {name}_ref_t_stop_enabled = false;
            end

            when {name}_resetting and {name}_ref_t.level == 0 then
                {name}.stop();
                {name}_resetting = false;
            end
        ''')
        core_message_fields = [
            f'{name}.idle',
            f'{name}.position',
            f'{name}_alarm.level',
            f'{name}_ref_t.level',
        ]
        super().__init__(min_position=min_position, max_position=max_position, robot_brain=robot_brain,
                         lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int = ChainAxis.DEFAULT_SPEED) -> None:
        try:
            await super().move_to(position, speed)
        except RuntimeError as e:
            raise Exception(e)
        self.log.info(f'>>>{self.name} is moving to {position}mm with speed {speed}...')
        steps = self.compute_steps(position)
        self.log.info(f'>>>steps: {steps}')
        self.log.info(f'>>>Sending move chain axis command to lizard...')
        await self.robot_brain.send(
            f'{self.name}.position({steps}, {speed}, 40000);'
        )
        await rosys.sleep(0.3)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False

        if not self.ref_t:
            raise RuntimeError('yaxis is not at top reference, reset first')

        try:
            self.log.info(f'{self.name} is referencing...')
            await self.reference_right()
            await self.reference_left()

            self.is_referenced = True
            self.log.info(f'{self.name} is referenced')
            return True
        except Exception as e:
            self.log.error(f'error while referencing: {e}')
            return False
        finally:
            await self.stop()
            await self.robot_brain.send(
                f'{self.name}_ref_l_is_referencing = false;'
                f'{self.name}_ref_r_is_referencing = false;'
                f'{self.name}_ref_t_stop_enabled = false;'
            )

    async def reference_right(self) -> bool:
        self.log.info('moving to ref_r...')
        await self.robot_brain.send(
            f'{self.name}_ref_l_is_referencing = false;'
            f'{self.name}_ref_r_is_referencing = true;'
            f'{self.name}_ref_t_stop_enabled = true;'
        )
        await rosys.sleep(0.2)
        # move to ref_r
        await self.robot_brain.send(
            f'{self.name}.speed({self.DEFAULT_SPEED / 4});'
        )
        while self.ref_t:
            await rosys.sleep(0.1)
        self.log.info('moving slowly out of  ref_r...')
        # move slowly out of ref_r
        await self.robot_brain.send(
            f'{self.name}_ref_t_stop_enabled = false;'
            f'{self.name}.speed(-{self.DEFAULT_SPEED / 10});'
        )
        while not self.ref_t:
            await rosys.sleep(0.1)
        # set tmp 0 position
        await self.robot_brain.send(
            f'{self.name}.position = 0;'
        )
        await rosys.sleep(0.5)
        self.log.info('moving out of ref_r by offset...')
        # move out of ref_r by offset
        await self.robot_brain.send(
            f'{self.name}_ref_r_is_referencing = false;'
            f'{self.name}.position(-{self.REF_OFFSET}, {self.DEFAULT_SPEED / 10}, 40000);'
        )
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')
        self.log.info('setting 0 position')

        # set 0 position
        await self.robot_brain.send(
            f'{self.name}.position = 0;'
        )

        await rosys.sleep(0.5)

    async def reference_left(self, first_time: bool = True) -> bool:
        self.log.info('moving to ref_l...')
        await self.robot_brain.send(
            f'{self.name}_ref_r_is_referencing = false;'
            f'{self.name}_ref_l_is_referencing = true;'
            f'{self.name}_ref_t_stop_enabled = true;'
        )
        await rosys.sleep(0.2)
        # move to ref_l
        await self.robot_brain.send(
            f'{self.name}_ref_t_stop_enabled = true;'
            f'{self.name}.speed(-{self.DEFAULT_SPEED / 4});'
        )
        while self.ref_t:
            await rosys.sleep(0.1)

        # move slowly out of ref_l
        await self.robot_brain.send(
            f'{self.name}_ref_t_stop_enabled = false;'
            f'{self.name}.speed({self.DEFAULT_SPEED / 10});'
        )
        while not self.ref_t:
            await rosys.sleep(0.1)
        # move out of ref_l by offset
        await self.robot_brain.send(
            f'{self.name}_ref_l_is_referencing = false;'
            f'{self.name}.position({self.steps + self.REF_OFFSET}, {self.DEFAULT_SPEED / 10}, 40000);'
        )
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')

        # save position
        await rosys.sleep(0.5)
        if first_time is True:
            self.steps_to_end = self.steps
            self.log.info(f'steps_to_end: {self.steps_to_end}')
        else:
            self.log.info('setting steps_to_end position')
            await self.robot_brain.send(
                f'{self.name}.position = {self.steps_to_end};'
            )
        await rosys.sleep(0.5)
        await self.robot_brain.send(
            f'{self.name}_ref_t_stop_enabled = false;'
        )
        await rosys.sleep(0.5)

    async def reset(self) -> bool:
        await super().reset()
        rosys.notify('chain axis will be reset in 10s, lift the front up!', type='warning')
        await rosys.sleep(10)
        await self.robot_brain.send(
            f'{self.name}_resetting = true;'
            f'{self.name}_ref_l_is_referencing = false;'
            f'{self.name}_ref_r_is_referencing = false;'
            f'{self.name}_ref_t_stop_enabled = false;'
            f'{self.name}.speed({self.DEFAULT_SPEED / 10});'
        )
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')
        self.is_referenced = False
        return True

    async def check_idle_or_alarm(self) -> bool:
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.info("zaxis alarm")
            return False
        return True

    async def move_dw_to_l_ref(self) -> None:
        try:
            await super().move_dw_to_l_ref()
        except RuntimeError as e:
            raise Exception(e)
        await self.robot_brain.send(
            f'{self.name}.position({(-self.steps_to_end + 4*self.REF_OFFSET)*self.TOP_DOWN_FACTOR}, {self.DEFAULT_SPEED}, 40000);'
        )
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')
        await self.reference_left(first_time=False)

    async def move_dw_to_r_ref(self) -> None:
        try:
            await super().move_dw_to_r_ref()
        except RuntimeError as e:
            raise Exception(e)
        await self.robot_brain.send(
            f'{self.name}.position({(self.steps_to_end*2 + -4*self.REF_OFFSET)*self.TOP_DOWN_FACTOR}, {self.DEFAULT_SPEED}, 40000);'
        )
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')
        await self.reference_right()

    async def return_to_l_ref(self) -> None:
        try:
            await super().return_to_l_ref()
        except RuntimeError as e:
            raise Exception(e)
        if self.steps <= self.steps_to_end + self.REF_OFFSET:
            return
        await self.robot_brain.send(
            f'{self.name}.position({self.steps_to_end}, {self.DEFAULT_SPEED/2}, 40000);'
        )
        await rosys.sleep(0.3)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')

    async def return_to_r_ref(self) -> None:
        try:
            await super().return_to_r_ref()
        except RuntimeError as e:
            raise Exception(e)
        if self.steps >= -self.REF_OFFSET:
            return
        await self.robot_brain.send(
            f'{self.name}.position(0, {self.DEFAULT_SPEED/2}, 40000);'
        )
        await rosys.sleep(0.3)
        if not await self.check_idle_or_alarm():
            raise Exception('chain_axis fault detected')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False
        self.ref_t = int(words.pop(0)) == 0


class ChainAxisSimulation(ChainAxis, rosys.hardware.ModuleSimulation):

    def __init__(self) -> None:
        super().__init__()
        self.speed: int = 0
        self.target_steps: Optional[int] = None
        self.reference_steps: int = 0
        self.ref_t = True

    async def stop(self) -> None:
        await super().stop()
        self.speed = 0
        self.target_steps = None

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.DEFAULT_SPEED
        try:
            await super().move_to(position, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        self.target_steps = self.compute_steps(position)
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def move_dw_to_l_ref(self, speed: int = ChainAxis.DEFAULT_SPEED) -> None:
        try:
            await super().move_dw_to_l_ref()
        except RuntimeError as e:
            rosys.notify(f'error while moving downwards to l ref: {e}', type='negative')
            return
        self.target_steps = self.steps_to_end
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def move_dw_to_r_ref(self, speed: int = ChainAxis.DEFAULT_SPEED) -> None:
        try:
            await super().move_dw_to_r_ref()
        except RuntimeError as e:
            rosys.notify(f'error while moving downwards to r ref: {e}', type='negative')
            return
        self.target_steps = 0
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def return_to_l_ref(self, speed: int = ChainAxis.DEFAULT_SPEED/4) -> None:
        try:
            await super().return_to_l_ref()
        except RuntimeError as e:
            rosys.notify(f'error while returning to l ref: {e}', type='negative')
            return
        self.target_steps = self.steps_to_end
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def return_to_r_ref(self, speed: int = ChainAxis.DEFAULT_SPEED/4) -> None:
        try:
            await super().return_to_r_ref()
        except RuntimeError as e:
            rosys.notify(f'error while returning to r ref: {e}', type='negative')
            return
        self.target_steps = 0
        self.speed = speed if self.target_steps > self.steps else -speed
        while self.target_steps is not None:
            await rosys.sleep(0.2)

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        self.steps = 0
        self.reference_steps = 0
        self.is_referenced = True
        return True

    async def step(self, dt: float) -> None:
        self.steps += int(dt * self.speed)
        self.idle = self.speed == 0
        if self.target_steps is not None and (self.speed > 0) == (self.steps > self.target_steps):
            self.steps = self.target_steps
            self.target_steps = None
            self.speed = 0
        self.ref_t = True
