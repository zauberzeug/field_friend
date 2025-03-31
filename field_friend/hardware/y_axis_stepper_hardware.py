# pylint: disable=duplicate-code
# TODO: refactor this and z_axis_stepper_hardware.py
import rosys
from rosys.helpers import remove_indentation

from ..config import YStepperConfiguration
from .axis import Axis


class YAxisStepperHardware(Axis, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, config: YStepperConfiguration, robot_brain: rosys.hardware.RobotBrain, *,
                 expander: rosys.hardware.ExpanderHardware | None) -> None:
        self.config = config
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {config.name}_motor = {expander.name + "." if config.motor_on_expander and expander else ""}StepperMotor({config.step_pin}, {config.direction_pin})
            {config.name}_alarm = {expander.name + "." if config.motor_on_expander and expander else ""}Input({config.alarm_pin})
            {config.name}_alarm.inverted = {str(config.alarm_inverted).lower()}
            {config.name}_end_l = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_left_pin})
            {config.name}_end_l.inverted = {str(config.end_stops_inverted).lower()}
            {config.name}_end_r = {expander.name + "." if config.end_stops_on_expander and expander else ""}Input({config.end_right_pin})
            {config.name}_end_r.inverted = {str(config.end_stops_inverted).lower()}
            {config.name} = {expander.name + "." if config.motor_on_expander and expander else ""}MotorAxis({config.name}_motor, {config.name + "_end_l" if config.reversed_direction else config.name + "_end_r"}, {config.name + "_end_r" if config.reversed_direction else config.name + "_end_l"})
        ''')
        core_message_fields = [
            f'{config.name}_end_left.active',
            f'{config.name}_end_right.active',
            f'{config.name}_motor.idle',
            f'{config.name}_motor.position',
            f'{config.name}_alarm.active'
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
        await self.robot_brain.send(f'{self.config.name}.stop()')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if not speed:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.info(f'could not move yaxis to {position} because of {error}')
            # TODO: we need a useful exception here
            raise Exception(  # pylint: disable=broad-exception-raised
                f'could not move yaxis to {position} because of {error}') from error
        steps = self.compute_steps(position)
        await self.robot_brain.send(f'{self.config.name}.position({steps}, {speed}, 250000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception  # pylint: disable=broad-exception-raised

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

            # if in end l stop, move out first
            if self.end_l:
                self.log.info('already in end_l moving out of end_l stop')
                velocity = self.reference_speed * \
                    (1 if self.reversed_direction else -1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity});')
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}.stop();')

            # if to end r stop if not already there
            if not self.end_r:
                self.log.info('moving to end_r stop')
                velocity = self.reference_speed * \
                    (1 if self.reversed_direction else -1)
                await self.robot_brain.send(f'{self.config.name}.speed({velocity});')
                while not self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.config.name}.stop();')

            # move out of end r stop
            self.log.info('moving out of end_r stop')
            velocity = self.reference_speed * \
                (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.config.name}.stop();')

            # move slowly to end r stop
            self.log.info('moving slowly to end_r stop')
            velocity = round(self.reference_speed / 2) * \
                (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move slowly out of end r stop
            self.log.info('moving slowly out of end_r stop')
            velocity = round(self.reference_speed / 2) * \
                (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.config.name}.speed({velocity});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.config.name}.stop();')

            # save position
            await rosys.sleep(0.5)
            await self.robot_brain.send(f'{self.config.name}_motor.position = 0;')
            await rosys.sleep(0.5)
            self.is_referenced = True
            return True
        except Exception as error:
            self.log.error(f'could not reference yaxis because of {error}')
            return False
        finally:
            await self.stop()

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.end_l = words.pop(0) == 'true'
        self.end_r = words.pop(0) == 'true'
        if self.end_l or self.end_r:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
