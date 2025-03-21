

import rosys
from rosys.helpers import remove_indentation


class DoubleUuWheelsHardware(rosys.hardware.Wheels, rosys.hardware.ModuleHardware):
    """This module implements wheels hardware.

    Drive and stop commands are forwarded to a given Robot Brain.
    Velocities are read and emitted regularly.
    """

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 name: str = 'wheels',
                 left_can_address: int = 12,
                 right_can_address: int = 11,
                 m_per_tick: float = 0.01,
                 width: float = 0.5,
                 is_left_reversed: bool = False,
                 is_right_reversed: bool = False,) -> None:
        self.name = name
        self.single_motor = left_can_address == right_can_address
        self.l_error = False
        self.r_error = False
        self.l_error_code1 = 0
        self.l_error_code2 = 0
        self.r_error_code1 = 0
        self.r_error_code2 = 0
        lizard_code = remove_indentation(f'''
            l = UUMotor({can.name}, {left_can_address},{3 if not self.single_motor else 1})
            r = UUMotor({can.name}, {right_can_address},{3 if not self.single_motor else 2})
            l.m_per_tick = {m_per_tick}
            r.m_per_tick = {m_per_tick}
            l.reversed = {'true' if is_left_reversed else 'false'}
            r.reversed = {'true' if is_right_reversed else 'false'}
            {name} = UUWheels(l, r)
            {name}.width = {width}
        ''')
        core_message_fields = [f'{self.name}.linear_speed:3',
                               f'{self.name}.angular_speed:3', 'l.error_flag', 'r.error_flag']
        if self.single_motor:
            core_message_fields.append('l.error_code')
            core_message_fields.append('r.error_code')
        else:
            core_message_fields.append('l.error_code1')
            core_message_fields.append('l.error_code2')
            core_message_fields.append('r.error_code1')
            core_message_fields.append('r.error_code2')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        if linear == 0.0:
            linear = -0.0
        if angular == 0.0:
            angular = -0.0  # TODO: Temp fix
        await self.robot_brain.send(f'{self.name}.speed({linear}, {angular})')

    async def reset_motors(self) -> None:
        if self.l_error:
            await self.robot_brain.send('l.reset_motor()')
        if self.r_error:
            await self.robot_brain.send('r.reset_motor()')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = rosys.geometry.Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        self.VELOCITY_MEASURED.emit([velocity])
        self.l_error = (words.pop(0) == 'true')
        self.r_error = (words.pop(0) == 'true')
        self.l_error_code1 = int(words.pop(0))
        if not self.single_motor:
            self.l_error_code2 = int(words.pop(0))
        self.r_error_code1 = int(words.pop(0))
        if not self.single_motor:
            self.r_error_code2 = int(words.pop(0))
