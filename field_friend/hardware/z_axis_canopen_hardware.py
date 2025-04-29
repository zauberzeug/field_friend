# pylint: disable=broad-exception-raised
# TODO: we need a useful exception here
import rosys
from rosys.helpers import remove_indentation

from ..config import ZCanOpenConfiguration
from .axis import Axis


class ZAxisCanOpenHardware(Axis, rosys.hardware.ModuleHardware):
    """Controls a vertical axis using a CANOpen motor."""

    def __init__(self, config: ZCanOpenConfiguration, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {config.name}_motor = {expander.name + "." if config.motor_on_expander and expander else ""}CanOpenMotor({can.name}, {config.can_address})
            {config.name}_end_t = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_top_pin})
            {config.name}_end_t.inverted = {str(config.end_stops_inverted).lower()}
            {config.name}_end_b = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_bottom_pin})
            {config.name}_end_b.inverted = {str(config.end_stops_inverted).lower()}
            {config.name} = {expander.name + "." if config.motor_on_expander and expander else ""}MotorAxis({config.name}_motor, {config.name + "_end_t" if config.reversed_direction else config.name + "_end_b"}, {config.name + "_end_b" if config.reversed_direction else config.name + "_end_t"})
        ''')
        core_message_fields = [
            f'{config.name}_end_t.active',
            f'{config.name}_end_b.active',
            f'{config.name}_motor.actual_position',
            f'{config.name}_motor.status_target_reached',
            f'{config.name}_motor.status_fault',
        ]
        super().__init__(
            max_speed=config.max_speed,
            reference_speed=config.reference_speed,
            min_position=config.min_position,
            max_position=config.max_position,
            axis_offset=config.axis_offset,
            steps_per_m=config.steps_per_m,
            reversed_direction=config.reversed_direction,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields)

    async def stop(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(false);')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.error(f'could not move zaxis to {position} because of {error}')
            raise Exception(f'could not move zaxis to {position} because of {error}') from error
        steps = self.compute_steps(position)
        self.log.debug(f'moving to steps: {steps}')
        await self.enable_motor()
        await rosys.sleep(0.1)
        await self.robot_brain.send(
            f'{self.config.name}.position({steps}, {speed}, 0);'
        )
        # Give flags time to turn false first
        await rosys.sleep(0.2)
        while not self.idle and not self.alarm:
            await self.robot_brain.send(
                f'{self.config.name}.position({steps}, {speed}, 0);'
            )
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.error(f'could not move zaxis to {position} because of fault')
            raise Exception(f'could not move zaxis to {position} because of fault')
        self.log.debug(f'zaxis moved to {position}')
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(false);')

    async def enable_motor(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(true);')

    async def disable_motor(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_enable(false);')

    async def reset_fault(self) -> None:
        await self.robot_brain.send(f'{self.config.name}_motor.reset_fault();')
        await rosys.sleep(1)
        if self.alarm:
            self.log.error('could not reset zaxis fault')
            raise Exception('could not reset zaxis fault')

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            self.log.info('enabling h motors')
            await self.enable_motor()
            await self.robot_brain.send(
                f'{self.config.name}_motor.position_offset = 0;'
            )
            await rosys.sleep(1)
            self.log.info('activating velocity mode')
            await self.robot_brain.send(
                f'{self.config.name}_motor.enter_pv_mode();'
            )
            await rosys.sleep(1)

            # if in end b stop, move out
            if self.end_b:
                self.log.info('already in end_b moving out of end_b stop')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.config.name}.speed({velocity}, 0);'
                )
                while self.end_b:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}_motor.set_ctrl_halt(true);')
            await rosys.sleep(0.5)

            # move to end t stop if not already there
            if not self.end_t:
                self.log.info('moving to end_t stop')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.config.name}.speed({velocity}, 0);'
                )
                while not self.end_t:
                    await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move out of end t stop
            self.log.info('moving out of end_t stop')
            velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.config.name}.speed({velocity}, 0);'
            )
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly to end t stop
            self.log.info('moving slowly to end_t stop')
            slow_velocity = 25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.config.name}.speed({slow_velocity}, 0);'
            )
            while not self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly out of end t stop
            self.log.info('moving slowly out of end_t stop')
            slow_velocity = -25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.config.name}.speed({slow_velocity}, 0);'
            )
            while self.end_t:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # save position
            await self.robot_brain.send(f'{self.config.name}_motor.position_offset = {self.steps};')
            await rosys.sleep(0.2)
            self.log.info('zaxis referenced')
            self.is_referenced = True
            self.log.info(f'actual position: {self.position}, and steps: {self.steps}')
            return True
        except Exception as error:
            self.log.error(f'could not reference zaxis because of {error}')
            return False
        finally:
            await self.stop()

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_t = words.pop(0) == 'true'
        self.end_b = words.pop(0) == 'true'
        if self.end_t or self.end_b:
            self.is_referenced = False
        self.steps = int(words.pop(0))
        self.idle = words.pop(0) == 'true'
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
