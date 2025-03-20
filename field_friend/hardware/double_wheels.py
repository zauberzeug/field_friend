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
                 is_right_reversed: bool = False,
                 odrive_version: int = 4,) -> None:
        self.name = name
        self.l0_error = 0
        self.r0_error = 0
        self.l1_error = 0
        self.r1_error = 0
        self.motor_error = False
        self.odrive_version = odrive_version
        lizard_code = remove_indentation(f'''
            l0 = ODriveMotor({can.name}, {left_back_can_address}{', 6'if self.odrive_version == 6 else ''})
            r0 = ODriveMotor({can.name}, {right_back_can_address}{', 6'if self.odrive_version == 6 else ''})
            l1 = ODriveMotor({can.name}, {left_front_can_address}{', 6'if self.odrive_version == 6 else ''})
            r1 = ODriveMotor({can.name}, {right_front_can_address}{', 6'if self.odrive_version == 6 else ''})
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
        if self.odrive_version == 6:
            core_message_fields.extend(['l0.motor_error_flag', 'r0.motor_error_flag',
                                       'l1.motor_error_flag', 'r1.motor_error_flag'])
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def drive(self, linear: float, angular: float, epsilon: float = 1e-6) -> None:
        if (abs(linear - self.linear_target_speed) < epsilon and
                abs(angular - self.angular_target_speed) < epsilon):
            return
        await super().drive(linear, angular)
        if linear == 0.0:
            linear = -0.0
        if angular == 0.0:
            angular = -0.0  # TODO: Temp fix
        await self.robot_brain.send(f'{self.name}.speed({linear}, {angular})')

    async def reset_motors(self) -> None:
        if not self.motor_error:
            return
        if self.l0_error == 1:
            await self.robot_brain.send('l0.reset_motor()')
        if self.r0_error == 1:
            await self.robot_brain.send('r0.reset_motor()')
        if self.l1_error == 1:
            await self.robot_brain.send('l1.reset_motor()')
        if self.r1_error == 1:
            await self.robot_brain.send('r1.reset_motor()')
        self.motor_error = False

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = rosys.geometry.Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        self.VELOCITY_MEASURED.emit([velocity])
        if self.odrive_version == 6:
            self.l0_error = int(words.pop(0))
            if self.l0_error == 1 and not self.motor_error:
                rosys.notify('Left Back Motor Error', 'warning')
                self.motor_error = True
            self.r0_error = int(words.pop(0))
            if self.r0_error == 1 and not self.motor_error:
                rosys.notify('Right Back Motor Error', 'warning')
                self.motor_error = True
            self.l1_error = int(words.pop(0))
            if self.l1_error == 1 and not self.motor_error:
                rosys.notify('Left Front Motor Error', 'warning')
                self.motor_error = True
            self.r1_error = int(words.pop(0))
            if self.r1_error == 1 and not self.motor_error:
                rosys.notify('Right Front Motor Error', 'warning')
                self.motor_error = True
