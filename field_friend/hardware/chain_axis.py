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
    DEFAULT_SPEED: float = 40000
    MIN_POSITION = -0.14235
    MAX_POSITION = 0.14235
    CHAIN_RADIUS = 0.05185
    WORK_OFFSET = 0.02

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

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
            steps = int(ramp(position, self.MIN_POSITION, self.CHAIN_RADIUS, 0, -16000, clip=True))
        elif self.MIN_POSITION + self.CHAIN_RADIUS <= position <= self.MAX_POSITION - self.CHAIN_RADIUS:
            steps = int(ramp(position, self.MIN_POSITION + self.CHAIN_RADIUS,
                        self.MAX_POSITION - self.CHAIN_RADIUS, -16000, self.steps_to_end + 16000, clip=True))
        else:
            steps = int(ramp(position, self.MAX_POSITION - self.CHAIN_RADIUS, self.MAX_POSITION,
                        self.steps_to_end + 16000, self.steps_to_end, clip=True))
        return steps

    def compute_position(self, steps: int) -> float:
        if steps > -16000:
            position = ramp(steps, 0, -16000, self.MIN_POSITION, self.MIN_POSITION + self.CHAIN_RADIUS)
        elif -16000 > steps >= self.steps_to_end + 16000:
            position = ramp(steps, -16000, self.steps_to_end + 16000,
                            self.MIN_POSITION + self.CHAIN_RADIUS, self.MAX_POSITION - self.CHAIN_RADIUS)
        else:
            position = ramp(steps, self.steps_to_end + 16000, self.steps_to_end,
                            self.MAX_POSITION - self.CHAIN_RADIUS, self.MAX_POSITION)
        return position

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)

    @abc.abstractmethod
    async def move_to(self, position: float, speed: int) -> None:
        if not self.is_referenced:
            raise RuntimeError('yaxis is not referenced, reference first')
        if not self.MIN_POSITION + self.WORK_OFFSET <= position <= self.MAX_POSITION - self.WORK_OFFSET:
            raise RuntimeError('target yaxis position is out of range')
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
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 13,
                 ref_l_pin: int = 14,
                 ref_r_pin: int = 35,
                 ref_t_pin: int = 21,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if motor_on_expander == True else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander == True else ""}Input({alarm_pin})
            {name}_ref_l = {expander.name + "." if end_stops_on_expander == True else ""}Input({ref_l_pin})
            {name}_ref_r = {expander.name + "." if end_stops_on_expander == True else ""}Input({ref_r_pin})
            {name}_ref_t = {expander.name + "." if end_stops_on_expander == True else ""}Input({ref_t_pin})

            bool {name}_ref_r_is_referencing = false
            bool {name}_ref_l_is_referencing = false
            bool {name}_ref_l_stop_enabled = true
            bool {name}_ref_r_stop_enabled = true
            bool {name}_ref_r_return = false
            bool {name}_ref_l_return = false
            bool {name}_resetting = false

            when {name}_ref_r_is_referencing and {name}_ref_r_stop_enabled and {name}_ref_r.level == 0 then
                {name}.stop();
            end

            when {name}_ref_l_is_referencing and {name}_ref_l_stop_enabled and {name}_ref_l.level == 0 then
                {name}.stop();
            end

            when {name}_ref_r_is_referencing and !{name}_ref_r_stop_enabled and {name}_ref_r.level == 1 then
                {name}.stop();
            end

            when {name}_ref_l_is_referencing and !{name}_ref_l_stop_enabled and {name}_ref_l.level == 1 then
                {name}.stop();
            end

            when {name}_ref_r_return and {name}_ref_t.level == 0 and {name}_ref_r.level == 0 then
                {name}.stop();
                {name}_ref_r_return = false;
            end

            when {name}_ref_l_return and {name}_ref_t.level == 0 and {name}_ref_l.level == 0 then
                {name}.stop();
                {name}_ref_l_return = false;
            end

            when {name}_resetting and {name}_ref_t.level == 0 and {name}_ref_r.level == 0 then
                {name}.stop();
                {name}_resetting = false;
            end
        ''')
        core_message_fields = [
            f'{name}.idle',
            f'{name}.position',
            f'{name}_alarm.level',
            f'{name}_ref_l.level',
            f'{name}_ref_r.level',
            f'{name}_ref_t.level',
        ]
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int = ChainAxis.DEFAULT_SPEED) -> None:
        try:
            await super().move_to(position, speed)
        except RuntimeError as e:
            rosys.notify(e, type='negative')
            return
        self.log.info(f'>>>{self.name} is moving to {position}mm with speed {speed}...')
        steps = self.compute_steps(position)
        self.log.info(f'>>>steps: {steps}')
        self.log.info(f'>>>Sending move chain axis command to lizard...')
        await self.robot_brain.send(
            f'{self.name}.position({steps}, {speed}, 40000);'
        )
        await rosys.sleep(0.3)
        if not await self.check_idle_or_alarm():
            rosys.notify('yaxis fault detected', type='negative')
            self.log.error(f'{self.name} fault detected')
            return

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False

        if not self.ref_t:
            rosys.notify(f'{self.name} is not at top reference, try to reset chain axis first!')
            return False

        try:
            self.log.info(f'{self.name} is referencing...')
            await self.robot_brain.send(f'{self.name}_is_referencing = true')

            # if in ref_l, move out first:
            if self.ref_l:
                await self.robot_brain.send(
                    f'{self.name}_ref_l_is_referencing = true;'
                    f'{self.name}_ref_l_stop_enabled = false;'
                    f'{self.name}.speed({self.DEFAULT_SPEED / 2});'
                )
                while self.ref_l:
                    await rosys.sleep(0.1)

            await self.robot_brain.send(
                f'{self.name}_ref_l_is_referencing = false;'
                f'{self.name}_ref_r_is_referencing = true;'
            )
            await rosys.sleep(0.2)

            # if already in ref_r, move out first:
            if self.ref_r:
                await self.robot_brain.send(
                    f'{self.name}_ref_r_stop_enabled = false;'
                    f'{self.name}.speed(-{self.DEFAULT_SPEED / 2});'
                )
                while self.ref_r:
                    await rosys.sleep(0.1)

            # move to ref_r
            await self.robot_brain.send(
                f'{self.name}_ref_r_stop_enabled = true;'
                f'{self.name}.speed({self.DEFAULT_SPEED / 2});'
            )
            while not self.ref_r:
                await rosys.sleep(0.1)

            # move slowly out of ref_r
            await self.robot_brain.send(
                f'{self.name}_ref_r_stop_enabled = false;'
                f'{self.name}.speed(-{self.DEFAULT_SPEED / 10});'
            )
            while self.ref_r:
                await rosys.sleep(0.1)

            # move slowly to ref_r
            await self.robot_brain.send(
                f'{self.name}_ref_r_stop_enabled = true;'
                f'{self.name}.speed({self.DEFAULT_SPEED / 10});'
            )
            while not self.ref_r:
                await rosys.sleep(0.1)

            # set 0 position
            await self.robot_brain.send(
                f'{self.name}.position = 0;'
            )
            await rosys.sleep(0.2)

            # move out of ref_r, to avoid ref stop
            await self.robot_brain.send(
                f'{self.name}_ref_r_stop_enabled = false;'
                f'{self.name}.speed(-{self.DEFAULT_SPEED / 10});'
            )
            while self.ref_r:
                await rosys.sleep(0.1)

            await self.robot_brain.send(
                f'{self.name}_ref_r_is_referencing = false;'
                f'{self.name}_ref_l_is_referencing = true;'
            )
            await rosys.sleep(0.2)

            # move to ref_l
            await self.robot_brain.send(
                f'{self.name}_ref_l_stop_enabled = true;'
                f'{self.name}.speed(-{self.DEFAULT_SPEED / 2});'
            )
            while not self.ref_l:
                await rosys.sleep(0.1)

            # move slowly out of ref_l
            await self.robot_brain.send(
                f'{self.name}_ref_l_stop_enabled = false;'
                f'{self.name}.speed({self.DEFAULT_SPEED / 10});'
            )
            while self.ref_l:
                await rosys.sleep(0.1)

            # move slowly to ref_l
            await self.robot_brain.send(
                f'{self.name}_ref_l_stop_enabled = true;'
                f'{self.name}.speed(-{self.DEFAULT_SPEED / 10});'
            )
            while not self.ref_l:
                await rosys.sleep(0.1)

            # save position
            await rosys.sleep(0.5)
            self.steps_to_end = self.steps
            self.log.info(f'steps_to_end: {self.steps_to_end}')

            await self.robot_brain.send(
                f'{self.name}_ref_l_is_referencing = false;'
                f'{self.name}_ref_l_stop_enabled = false;'
                f'{self.name}_ref_r_stop_enabled = false;'
            )
            await rosys.sleep(0.5)
            self.is_referenced = True
            self.log.info(f'{self.name} is referenced')
            return True
        except Exception as e:
            self.log.error(f'error while referencing: {e}')
            return False
        finally:
            await self.stop()

    async def reset(self) -> bool:
        await super().reset()
        rosys.notify('chain axis will be reset in 10s, lift the front up!', type='warning')
        await rosys.sleep(10)
        await self.robot_brain.send(
            f'{self.name}_resetting = true;'
            f'{self.name}.speed(-{self.DEFAULT_SPEED / 10});'
        )
        if not await self.check_idle_or_alarm():
            rosys.notify('chain_axis fault detected', type='negative')
            return False
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
            rosys.notify(f'error while moving down to l ref: {e}', type='negative')
            return
        await self.robot_brain.send(
            f'{self.name}_ref_l_return = true;'
            f'{self.name}.speed({self.DEFAULT_SPEED}, 40000);'
        )
        while not self.ref_l:
            await rosys.sleep(0.1)
        await rosys.sleep(0.2)
        self.end_position = self.steps
        await self.robot_brain.send(
            f'{self.name}_ref_l_return = false;'
        )
        await rosys.sleep(0.2)
        await self.return_to_l_ref()

    async def move_dw_to_r_ref(self) -> None:
        try:
            await super().move_dw_to_r_ref()
        except RuntimeError as e:
            rosys.notify(f'error while moving down to r ref: {e}', type='negative')
            return
        await self.robot_brain.send(
            f'{self.name}_ref_r_return = true;'
            f'{self.name}.speed(-{self.DEFAULT_SPEED}, 40000);'
        )
        while not self.ref_r:
            await rosys.sleep(0.1)
        await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}_ref_r_return = false;'
        )
        await rosys.sleep(0.2)
        await self.return_to_r_ref()

    async def return_to_l_ref(self) -> None:
        try:
            await super().return_to_l_ref()
        except RuntimeError as e:
            rosys.notify(f'error while returning to l ref: {e}', type='negative')
            return
        await self.robot_brain.send(
            f'{self.name}_ref_l_return = true;'
            f'{self.name}.speed(-{self.DEFAULT_SPEED/3}, 20000);'
        )
        while not self.ref_l or not self.ref_t:
            await rosys.sleep(0.1)
        await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}_ref_l_return = false;'
        )
        await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}.position = {self.steps_to_end};'
        )

    async def return_to_r_ref(self) -> None:
        try:
            await super().return_to_r_ref()
        except RuntimeError as e:
            rosys.notify(f'error while returning to r ref: {e}', type='negative')
            return
        await self.robot_brain.send(
            f'{self.name}_ref_r_return = true;'
            f'{self.name}.speed({self.DEFAULT_SPEED/3}, 20000);'
        )
        while not self.ref_r or not self.ref_t:
            await rosys.sleep(0.1)
        await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}_ref_r_return = false;'
        )
        await rosys.sleep(0.2)
        await self.robot_brain.send(
            f'{self.name}.position = 0;'
        )

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False
        self.ref_l = int(words.pop(0)) == 0
        self.ref_r = int(words.pop(0)) == 0
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

    async def move_to(self, position: float, speed: int = ChainAxis.DEFAULT_SPEED) -> None:
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
        if self.steps == 0:
            self.ret_t = True
            self.ref_r = True
        elif self.steps == self.steps_to_end:
            self.ret_t = True
            self.ref_l = True
        else:
            self.ret_t = True
            self.ref_l = False
            self.ref_r = False
