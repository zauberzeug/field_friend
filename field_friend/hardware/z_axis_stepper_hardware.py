# pylint: disable=broad-exception-raised
# TODO: we need a useful exception here
import rosys
from rosys.helpers import remove_indentation

from ..config import ZStepperConfiguration
from .axis import Axis


class ZAxisStepperHardware(Axis, rosys.hardware.ModuleHardware):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, config: ZStepperConfiguration,
                 robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander

        lizard_code = remove_indentation(f'''
            {self.config.name}_motor = {expander.name + "." if self.config.motor_on_expander and expander else ""}StepperMotor({self.config.step_pin}, {self.config.direction_pin})
            {self.config.name}_alarm = {expander.name + "." if self.config.motor_on_expander and expander else ""}Input({self.config.alarm_pin})
            {self.config.name}_end_t = {expander.name + "." if self.config.end_stops_on_expander and expander else ""}Input({self.config.end_top_pin})
            {self.config.name}_end_t.inverted = {str(self.config.end_stops_inverted).lower()}
            {self.config.name}_end_b = {expander.name + "." if self.config.end_stops_on_expander and expander else ""}Input({self.config.end_bottom_pin})
            {self.config.name}_end_b.inverted = {str(self.config.end_stops_inverted).lower()}
            {self.config.name} = {expander.name + "." if self.config.motor_on_expander and expander else ""}MotorAxis({self.config.name}_motor, {self.config.name + "_end_t" if self.config.reversed_direction else self.config.name + "_end_b"}, {self.config.name + "_end_b" if self.config.reversed_direction else self.config.name + "_end_t"})
        ''')
        core_message_fields = [
            f'{self.config.name}_end_t.active',
            f'{self.config.name}_end_b.active',
            f'{self.config.name}_motor.idle',
            f'{self.config.name}_motor.position',
            f'{self.config.name}_alarm.level',
        ]
        super().__init__(
            max_speed=self.config.max_speed,
            reference_speed=self.config.reference_speed,
            min_position=self.config.min_position,
            max_position=self.config.max_position,
            axis_offset=self.config.axis_offset,
            steps_per_m=self.config.steps_per_m,
            reversed_direction=self.config.reversed_direction,
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields
        )

    async def stop(self) -> None:
        await self.robot_brain.send(f'{self.config.name}.stop()')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.error(f'could not move zaxis to {position} because of {error}')
            raise Exception(f'could not move zaxis to {position} because of {error}') from error
        steps = self.compute_steps(position)

        await self.robot_brain.send(f'{self.config.name}.position({steps}, {speed}, 160000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('z_axis fault detected')
        self.log.debug(f'zaxis moved to {position}')

    async def check_idle_or_alarm(self) -> bool:
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.info('zaxis alarm')
            return False
        return True

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            self.log.info('zaxis referencing...')

            # if in end b stop, move out first
            if self.end_b:
                self.log.info('already in end b stop, moving out of it')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity},0);')
                while self.end_b:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}.stop();')

            # move to end t stop if not already there
            if not self.end_t:
                self.log.info('moving to end t stop')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity},0);')
                while not self.end_t:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}.stop();')

            # move out of end t stop
            self.log.info('moving out of end t stop')
            velocity = self.reference_speed * (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity},0);')
            while self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.config.name}.stop();')

            # move slowly to end t stop
            self.log.info('moving slowly to end t stop')
            velocity = round(self.reference_speed/2) * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity},0);')
            while not self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.config.name}.stop();')

            # move slowly out of end t stop
            self.log.info('moving slowly out of end t stop')
            velocity = round(self.reference_speed/2) * (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity},0);')
            while self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.config.name}.stop();')

            # save position
            await rosys.sleep(0.5)
            await self.robot_brain.send(
                f'{self.config.name}_motor.position = 0;'
            )
            await rosys.sleep(0.5)
            self.log.info('zaxis referenced')
            self.is_referenced = True
            return True
        except Exception as e:
            self.log.error(f'zaxis reference failed: {e}')
            return False
        finally:
            await self.stop()

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_t = words.pop(0) == 'true'
        self.end_b = words.pop(0) == 'true'
        if self.end_b:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
