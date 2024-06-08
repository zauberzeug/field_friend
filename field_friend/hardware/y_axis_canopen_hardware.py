from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .y_axis import YAxis


class YAxisCanOpenHardware(YAxis, rosys.hardware.ModuleHardware):
    """The y axis hardware module is a simple example for a representation of real robot hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'yaxis',
                 can: rosys.hardware.CanHardware,
                 expander: Optional[rosys.hardware.ExpanderHardware],
                 can_address: int = 0x60,
                 max_speed: int = 2000,
                 reference_speed: int = 40,
                 min_position: float = -0.068,
                 max_position: float = 0.068,
                 axis_offset: float = 0.075,
                 steps_per_m: float = 1_481_481.48,  # [steps/turn] / ([gear] * [m/turn])
                 end_r_pin: int = 19,
                 end_l_pin: int = 21,
                 motor_on_expander: bool = False,
                 end_stops_on_expander: bool = True,
                 end_stops_inverted: bool = False,
                 reversed_direction: bool = False,
                 acceleration: int = 500,
                 quick_stop_deceleration: int = 2000,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.ctrl_enable = False
        lizard_code = remove_indentation(f'''
            {name}_motor = {expander.name + "." if motor_on_expander and expander else ""}CanOpenMotor({can.name}, {can_address})
            {name}_end_l = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_l_pin})
            {name}_end_l.inverted = {str(end_stops_inverted).lower()}
            {name}_end_r = {expander.name + "." if end_stops_on_expander and expander else ""}Input({end_r_pin})
            {name}_end_r.inverted = {str(end_stops_inverted).lower()}
            {name} = {expander.name + "." if motor_on_expander and expander else ""}MotorAxis({name}_motor, {name + "_end_l" if reversed_direction else name + "_end_r"}, {name + "_end_r" if reversed_direction else name + "_end_l"})
        ''')
        core_message_fields = [
            f'{name}_end_l.active',
            f'{name}_end_r.active',
            f'{name}_motor.actual_position',
            f'{name}_motor.status_target_reached',
            f'{name}_motor.status_fault',
            f'{name}_motor.ctrl_enable',
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
            self.log.error(f'could not move yaxis to {position} because of {error}')
            raise Exception(f'could not move yaxis to {position} because of {error}')
        steps = self.compute_steps(position)
        self.log.info(f'moving to steps: {steps}')
        await self.enable_motor()
        await rosys.sleep(1)  # necessary ?!
        await self.robot_brain.send(
            f'{self.name}.position({steps},{speed}, 0);'
        )
        # Give flags time to turn false first
        await rosys.sleep(0.5)
        while not self.idle and not self.alarm:
            await rosys.sleep(0.2)
        if self.alarm:
            self.log.error(f'could not move yaxis to {position} because of fault')
            raise Exception(f'could not move yaxis to {position} because of fault')
        self.log.info(f'yaxis moved to {position}')
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_halt(true);')

    async def enable_motor(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(true);')

    async def disable_motor(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.set_ctrl_enable(false);')

    async def reset_fault(self) -> None:
        await self.robot_brain.send(f'{self.name}_motor.reset_fault();')
        await rosys.sleep(1)
        if self.alarm:
            self.log.error('could not reset yaxis fault')
            raise Exception('could not reset yaxis fault')

    async def try_reference(self) -> bool:
        if not await super().try_reference():
            return False
        try:
            self.log.info("enabling h motors")
            await self.enable_motor()
            await self.robot_brain.send(
                f'{self.name}_motor.position_offset = 0;'
            )
            await rosys.sleep(1)
            self.log.info('activating velocity mode')
            await self.robot_brain.send(
                f'{self.name}.speed(0, 0);'
            )
            await rosys.sleep(1)

            # if in end l stop, move out
            if self.end_l:
                self.log.info('already in end_l moving out of end_l stop')
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.name}.speed({velocity}, 0);'
                )
                while self.end_l:
                    await rosys.sleep(0.2)
                await self.robot_brain.send(f'{self.name}_motor.set_ctrl_halt(true);')
            await rosys.sleep(0.5)

            # move to end r stop if not already there
            if not self.end_r:
                self.log.info('moving to end_r stop')
                velocity = -self.reference_speed * (-1 if self.reversed_direction else 1)
                await self.robot_brain.send(
                    f'{self.name}.speed({velocity}, 0);'
                )
                while not self.end_r:
                    await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move out of end r stop
            self.log.info('moving out of end_r stop')
            velocity = self.reference_speed * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}.speed({velocity}, 0);'
            )
            while self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly to end r stop
            self.log.info('moving slowly to end_r stop')
            slow_velocity = -25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}.speed({slow_velocity}, 0);'
            )
            while not self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # move slowly out of end r stop
            self.log.info('moving slowly out of end_r stop')
            slow_velocity = 25 * (-1 if self.reversed_direction else 1)
            await self.robot_brain.send(
                f'{self.name}.speed({slow_velocity}, 0);'
            )
            while self.end_r:
                await rosys.sleep(0.2)
            await rosys.sleep(0.5)

            # check if this is necessary when enterning pp mode with negative speed
            await self.robot_brain.send(f'{self.name}_motor.enter_pv_mode(0);')

            # save position
            await self.robot_brain.send(f'{self.name}_motor.position_offset = {self.steps};')
            await rosys.sleep(0.2)
            self.log.info('yaxis referenced')
            self.is_referenced = True
            self.log.info(f'actual position: {self.position}, and steps: {self.steps}')
            await self.robot_brain.send(f'{self.name}_motor.enter_pp_mode(0);')
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
        self.steps = int(words.pop(0))
        self.idle = words.pop(0) == 'true'
        self.alarm = words.pop(0) == 'true'
        if self.alarm:
            self.is_referenced = False
        self.ctrl_enable = words.pop(0) == 'true'
