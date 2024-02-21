

import rosys
from rosys.helpers import remove_indentation


class DoubleWheelsHardware(rosys.hardware.Wheels, rosys.hardware.ModuleHardware):
    """This module implements wheels hardware.

    Drive and stop commands are forwarded to a given Robot Brain.
    Velocities are read and emitted regularly.
    """

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 name: str = 'wheels',
                 left_back_can_address: int = 0x000,
                 left_front_can_address: int = 0x100,
                 right_back_can_address: int = 0x200,
                 right_front_can_address: int = 0x300,
                 m_per_tick: float = 0.01,
                 width: float = 0.5,
                 is_left_reversed: bool = False,
                 is_right_reversed: bool = False,) -> None:
        self.name = name
        lizard_code = remove_indentation(f'''
            l0 = ODriveMotor({can.name}, {left_back_can_address})
            r0 = ODriveMotor({can.name}, {right_back_can_address})
            l1 = ODriveMotor({can.name}, {left_front_can_address})
            r1 = ODriveMotor({can.name}, {right_front_can_address})
            l0.m_per_tick = {m_per_tick}
            r0.m_per_tick = {m_per_tick}
            l1.m_per_tick = {m_per_tick}
            r1.m_per_tick = {m_per_tick}
            l0.reversed = {'true' if is_left_reversed else 'false'}
            r0.reversed = {'true' if is_right_reversed else 'false'}
            l1.reversed = {'true' if is_left_reversed else 'false'}
            r1.reversed = {'true' if is_right_reversed else 'false'}
            {name} = ODriveWheels(l0, r0)
            {name}_front = ODriveWheels(l1, r1)
            {name}.width = {width}
            {name}_front.width = {width}
            {name}.shadow({name}_front)
        ''')
        core_message_fields = [f'{self.name}.linear_speed:3', f'{self.name}.angular_speed:3']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        if linear == 0.0:
            linear = -0.0
        if angular == 0.0:
            angular = -0.0  # TODO: Temp fix
        await self.robot_brain.send(f'{self.name}.speed({linear}, {angular})')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = rosys.geometry.Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        self.VELOCITY_MEASURED.emit([velocity])
