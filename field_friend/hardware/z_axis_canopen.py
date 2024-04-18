import abc
from typing import Optional

import rosys
from rosys.helpers import remove_indentation


class ZAxisCanOpen(rosys.hardware.Module, abc.ABC):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, max_speed, reference_speed, min_position, max_position, axis_offset, steps_per_m, reversed_direction, **kwargs) -> None:
        super().__init__(**kwargs)

        self.max_speed: int = max_speed
        self.reference_speed: int = reference_speed
        self.min_position: float = min_position
        self.max_position: float = max_position
        self.axis_offset: float = axis_offset
        self.steps_per_m: float = steps_per_m
        self.reversed_direction: bool = reversed_direction

        self.steps: int = 0
        self.alarm: bool = False
        self.idle: bool = False

        self.is_referenced: bool = False

        self.end_t: bool = False
        self.end_b: bool = False

        rosys.on_shutdown(self.stop)

    @abc.abstractmethod
    async def stop(self) -> None:
        pass

    @abc.abstractmethod
    async def move_to(self, position: float, speed: int) -> None:
        if not self.is_referenced:
            raise RuntimeError('zaxis is not referenced, reference first')
        if speed > self.max_speed:
            raise RuntimeError(f'zaxis speed is too high, max speed is {self.max_speed}')
        if not self.min_position <= position <= self.max_position:
            raise RuntimeError(
                f'target position {position} ist out of zaxis range {self.min_position} to {self.max_position}')
        self.log.info(f'moving zaxis to {position} with speed {speed}')

    @abc.abstractmethod
    async def try_reference(self) -> bool:
        return True

    def compute_steps(self, position: float) -> int:
        """Compute the number of steps to move the y axis to the given position.   

        The position is given in meters.
        """
        return int((position + self.axis_offset) * self.steps_per_m) * (-1 if self.reversed_direction else 1)

    def compute_position(self, steps: int) -> float:
        return steps / self.steps_per_m - self.axis_offset * (-1 if self.reversed_direction else 1)

    @property
    def position(self) -> float:
        return self.compute_position(self.steps)

    async def return_to_reference(self) -> bool:
        try:
            await self.move_to(0)
        except RuntimeError as e:
            rosys.notify(e, type='negative')


class ZAxisCanOpenHardware(ZAxisCanOpen, rosys.hardware.ModuleHardware):
    """The z axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'zaxis',
                 can: rosys.hardware.CanHardware,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 can_address: int = 0x60,
                 max_speed: int = 2000,
                 reference_speed: int = 40,
                 min_position: float = -0.15,
                 max_position: float = 0.0,
                 axis_offset: float = 0.0,
                 steps_per_m: float = 1_481_481.48,  # 4000steps/turn motor; 1/20 gear; 0.054m/u
                 end_t_pin: int = 19,
                 end_b_pin: int = 21,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 reversed_direction: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            master = CanOpenMaster({can.name})
            {name}_motor = {expander.name + "." if motor_on_expander and expander else ""}CanOpenMotor({can.name}, {can_address})
            master.sync_interval = 5
            {name}_end_t = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_t_pin})
            {name}_end_b = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_b_pin})
            bool {name}_ends_enabled = true;
            bool {name}_is_referencing = false;
            when {name}_ends_enabled and ({name}_end_t.level == 0 or {name}_end_b.level == 0) then
                {name}_motor.set_ctrl_halt(true);
            end
            when !{name}_ends_enabled and {name}_is_referencing and {name}_end_t.level == 1 then
                {name}_motor.set_ctrl_halt(true);
            end
        ''')
        core_message_fields = [
            f'{name}_end_t.level',
            f'{name}_end_b.level',
            f'{name}_motor.actual_position',
            f'{name}_motor.status_target_reached',
            f'{name}_motor.status_fault',
        ]
        super().__init__(
            max_speed=max_speed,
            reference_speed=reference_speed,
            min_position=min_position,
            max_position=max_position,
            axis_offset=axis_offset,
            steps_per_m=steps_per_m,
            reversed_direction=reversed_direction,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(false);')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.error(f'could not move zaxis to {position} because of {error}')
            raise Exception(f'could not move zaxis to {position} because of {error}')
        steps = self.compute_steps(position)
        self.log.info(f'moving to steps: {steps}')
        await self.enable_motor()
        await self.enter_pp_mode(speed)
        await rosys.sleep(0.1)
        await self.robot_brain.send(
            f'{self.name}_motor.set_target_position({steps});'
            f'{self.name}_motor.commit_target_position();'
        )
        # Give flags time to turn false first
        await rosys.sleep(0.2)
        while not self.idle and not self.alarm:
            await self.enter_pp_mode(speed)
            await self.robot_brain.send(
                f'{self.name}_motor.set_target_position({steps});'
                f'{self.name}_motor.commit_target_position();'
            )
            await rosys.sleep(0.4)
        if self.alarm:
            self.log.error(f'could not move zaxis to {position} because of fault')
            raise Exception(f'could not move zaxis to {position} because of fault')
        self.log.info(f'zaxis moved to {position}')
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(false);')

    async def enable_motor(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(true);')

    async def disable_motor(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(false);')

    async def enter_pp_mode(self, velocity: int) -> None:
        await self.robot_brain.send(f'{self.name}_motor.enter_pp_mode({velocity});')

    async def check_target_reached_or_fault(self) -> bool:
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.error('zaxis fault')
            return False
        return True

    async def reset_fault(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.reset_fault();')
        await rosys.sleep(1)
        if self.alarm:
            self.log.error('could not reset zaxis fault')
            raise Exception('could not reset zaxis fault')

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            self.log.info("enabling h motors")
            await self.enable_motor()
            await self.robot_brain.send(
                f'{self.name}_is_referencing = true;'
                f'{self.name}_ends_enabled = true;'
                f'{self.name}_motor.position_offset = 0;'
            )
            await rosys.sleep(1)

            # if in end b stop, move out
            if self.end_b:
                self.log.info('already in end_b moving out of end_b stop')
                await self.robot_brain.send(f'{self.name}_ends_enabled = false;')
                await rosys.sleep(1)
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.name}_motor.enter_pv_mode({velocity});'
                    f'{self.name}_motor.set_ctrl_halt(false);'
                )
                while self.end_b:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}_motor.set_ctrl_halt(true);')
            await rosys.sleep(0.5)

            # move to end t stop if not already there
            if not self.end_t:
                self.log.info('moving to end_t stop')
                await self.robot_brain.send(f'{self.name}_ends_enabled = true;')
                await rosys.sleep(1)
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.name}_motor.enter_pv_mode({velocity});'
                    f'{self.name}_motor.set_ctrl_halt(false);'
                )
                while not self.end_t:
                    await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move out of end t stop
            self.log.info('moving out of end_t stop')
            await self.robot_brain.send(f'{self.name}_ends_enabled = false;')
            await rosys.sleep(1)
            velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}_motor.enter_pv_mode({velocity});'
                f'{self.name}_motor.set_ctrl_halt(false);'
            )
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly to end t stop
            self.log.info('moving slowly to end_t stop')
            await self.robot_brain.send(f'{self.name}_ends_enabled = true;')
            await rosys.sleep(1)
            slow_velocity = -0.5 * self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}_motor.enter_pv_mode({slow_velocity});'
                f'{self.name}_motor.set_ctrl_halt(false);'
            )
            while not self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly out of end t stop
            self.log.info('moving slowly out of end_t stop')
            await self.robot_brain.send(f'{self.name}_ends_enabled = false;')
            await rosys.sleep(1)
            slow_velocity = 0.5 * self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}_motor.enter_pv_mode({slow_velocity});'
                f'{self.name}_motor.set_ctrl_halt(false);'
            )
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # save position
            await self.robot_brain.send(f'{self.name}_motor.position_offset = {self.steps};')
            await rosys.sleep(0.2)
            await self.robot_brain.send(
                f'{self.name}_is_referencing = false;'
                f'{self.name}_ends_enabled = true;'
            )
            self.log.info('zaxis referenced')
            self.is_referenced = True
            self.log.info(f'actual position: {self.position}, and steps: {self.steps}')
            await self.move_to(0)
            return True
        except Exception as error:
            self.log.error(f'could not reference zaxis because of {error}')
            return False
        finally:
            await self.stop()
            await self.robot_brain.send(
                f'{self.name}_is_referencing = false;'
                f'{self.name}_ends_enabled = true;'
            )

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_t = int(words.pop(0)) == 0
        self.end_b = int(words.pop(0)) == 0
        if self.end_t or self.end_b:
            self.is_referenced = False
        self.steps = int(words.pop(0))
        self.idle = words.pop(0) == 'true'
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False


class ZAxisCanOpenSimulation(ZAxisCanOpen, rosys.hardware.ModuleSimulation):
    '''The z axis simulation module is a simple example for a representation of simulated robot hardware.
    '''

    def __init__(self,
                 max_speed: int = 80_000,
                 reference_speed: int = 40,
                 min_position: float = -0.12,
                 max_position: float = 0.12,
                 axis_offset: float = 0.123,
                 steps_per_m: float = 666.67 * 1000,
                 reversed_direction: bool = False,
                 ) -> None:
        self.speed: int = 0
        self.target_steps: Optional[int] = None
        super().__init__(
            max_speed=max_speed,
            reference_speed=reference_speed,
            min_position=min_position,
            max_position=max_position,
            axis_offset=axis_offset,
            steps_per_m=steps_per_m,
            reversed_direction=reversed_direction,
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
            self.log.error(f'could not move zaxis to {position} because of {e}')
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

    async def reset_fault(self) -> None:
        self.alarm = False

    async def step(self, dt: float) -> None:
        await super().step(dt)
        self.steps += int(dt * self.speed)
        self.idle = self.speed == 0
        if self.target_steps is not None:
            if (self.speed > 0) == (self.steps > self.target_steps):
                self.steps = self.target_steps
                self.target_steps = None
                self.speed = 0
