from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .y_axis import YAxis


class YAxisStepperHardware(YAxis, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'yaxis_tornado',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 max_speed: int = 80_000,
                 reference_speed: int = 15_000,
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
                 end_stops_inverted: bool = False,
                 reversed_direction: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander
        lizard_code = remove_indentation(f'''
            {name}_motor = {expander.name + "." if motor_on_expander and expander else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander and expander else ""}Input({alarm_pin})
            {name}_end_l = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_l_pin})
            {name}_end_l.inverted = {str(end_stops_inverted).lower()}
            {name}_end_r = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_r_pin})
            {name}_end_r.inverted = {str(end_stops_inverted).lower()}
            {name} = {expander.name + "." if motor_on_expander and expander else ""}MotorAxis({name}_motor, {name + "_end_l" if reversed_direction else name + "_end_r"}, {name + "_end_r" if reversed_direction else name + "_end_l"})
        ''')
        core_message_fields = [
            f'{name}_end_l.active',
            f'{name}_end_r.active',
            f'{name}_motor.idle',
            f'{name}_motor.position',
            f'{name}_alarm.active'
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
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if not speed:
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

            # if in end l stop, move out first
            if self.end_l:
                self.log.info('already in end_l moving out of end_l stop')
                velocity = self.reference_speed * (1 if self.reversed_direction else -1)
                await self.robot_brain.send(f'{self.name}.speed({velocity});')
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # if to end r stop if not already there
            if not self.end_r:
                self.log.info('moving to end_r stop')
                velocity = self.reference_speed * (1 if self.reversed_direction else -1)
                await self.robot_brain.send(f'{self.name}.speed({velocity});')
                while not self.end_r:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # move out of end r stop
            self.log.info('moving out of end_r stop')
            velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # move slowly to end r stop
            self.log.info('moving slowly to end_r stop')
            velocity = round(self.reference_speed / 2) * (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while not self.end_r:
                await rosys.sleep(0.2)

            # move slowly out of end r stop
            self.log.info('moving slowly out of end_r stop')
            velocity = round(self.reference_speed / 2) * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while self.end_r:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # save position
            await rosys.sleep(0.5)
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
        self.end_l = words.pop(0) == 'true'
        self.end_r = words.pop(0) == 'true'
        if self.end_l or self.end_r:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
