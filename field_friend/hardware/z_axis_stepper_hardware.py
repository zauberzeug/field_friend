from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .z_axis import ZAxis


class ZAxisStepperHardware(ZAxis, rosys.hardware.ModuleHardware):
    """The z axis module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'zaxis',
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 max_speed: float = 60_000,
                 reference_speed: float = 20_000,
                 min_position: float = -0.197,
                 max_position: float = 0.00,
                 axis_offset: float = 0.0,
                 steps_per_m: float = 1600 * 1000,
                 step_pin: int = 5,
                 dir_pin: int = 4,
                 alarm_pin: int = 33,
                 end_t_pin: int = 25,
                 end_b_pin: int = 22,
                 motor_on_expander: bool = True,
                 end_stops_on_expander: bool = True,
                 reversed_direction: bool = False,
                 ) -> None:
        self.name = name
        self.expander = expander

        lizard_code = remove_indentation(f'''
            {name}_motor = {expander.name + "." if motor_on_expander and expander else ""}StepperMotor({step_pin}, {dir_pin})
            {name}_alarm = {expander.name + "." if motor_on_expander and expander else ""}Input({alarm_pin})
            {name}_end_t = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_t_pin})
            {name}_end_b = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_b_pin})
            {name} =  {expander.name + "." if motor_on_expander and expander else ""}MotorAxis({name}_motor, {name + "_end_t" if reversed_direction else name + "_end_b"}, {name + "_end_b" if reversed_direction else name + "_end_t"})
        ''')
        core_message_fields = [
            f'{name}_end_t.level',
            f'{name}_end_b.level',
            f'{name}_motor.idle',
            f'{name}_motor.position',
            f'{name}_alarm.level',
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
            core_message_fields=core_message_fields
        )

    async def stop(self) -> None:
        await super().stop()
        await self.robot_brain.send(f'{self.name}.stop()')

    async def move_to(self, position: float, speed: int | None = None) -> None:
        if speed is None:
            speed = self.max_speed
        try:
            await super().move_to(position, speed)
        except RuntimeError as error:
            self.log.error(f'could not move zaxis to {position} because of {error}')
            raise Exception(f'could not move zaxis to {position} because of {error}') from error
        steps = self.compute_steps(position)

        await self.robot_brain.send(f'{self.name}.position({steps}, {speed}, 160000);')
        await rosys.sleep(0.2)
        if not await self.check_idle_or_alarm():
            raise Exception('z_axis fault detected')
        self.log.info(f'zaxis moved to {position}')

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

            # if in end b stop, move out first
            if self.end_b:
                self.log.info('already in end b stop, moving out of it')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.name}.speed({velocity});')
                while self.end_b:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # move to end t stop if not already there
            if not self.end_t:
                self.log.info('moving to end t stop')
                velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(f'{self.name}.speed({velocity});')
                while not self.end_t:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}.stop();')

            # move out of end t stop
            self.log.info('moving out of end t stop')
            velocity = self.reference_speed * (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # move slowly to end t stop
            self.log.info('moving slowly to end t stop')
            velocity = round(self.reference_speed/2) * (1 if self.reversed_direction else -1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while not self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # move slowly out of end t stop
            self.log.info('moving slowly out of end t stop')
            velocity = round(self.reference_speed/2) * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(f'{self.name}.speed({velocity});')
            while self.end_t:
                await rosys.sleep(0.2)
            await self.robot_brain.send(f'{self.name}.stop();')

            # save position
            await rosys.sleep(0.5)
            await self.robot_brain.send(
                f'{self.name}_motor.position = 0;'
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
        self.end_t = int(words.pop(0)) == 0
        self.end_b = int(words.pop(0)) == 0
        if self.end_b:
            self.is_referenced = False
        self.idle = words.pop(0) == 'true'
        self.steps = int(words.pop(0))
        self.alarm = int(words.pop(0)) == 0
        if self.alarm:
            self.is_referenced = False
