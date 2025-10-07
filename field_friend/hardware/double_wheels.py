import rosys
from rosys.helpers import remove_indentation

from ..config import WheelsConfiguration


class DoubleWheelsHardware(rosys.hardware.Wheels, rosys.hardware.ModuleHardware):
    """Expands the RoSys wheels hardware to control the field friend's tracked wheels with dual motors."""
    MAX_VALID_LINEAR_VELOCITY = 2.0

    def __init__(self, config: WheelsConfiguration, robot_brain: rosys.hardware.RobotBrain, estop: rosys.hardware.EStopHardware, *,
                 can: rosys.hardware.CanHardware,
                 m_per_tick: float = 0.01,
                 width: float = 0.5) -> None:
        self.config = config
        self.l0_error = 0
        self.r0_error = 0
        self.l1_error = 0
        self.r1_error = 0
        self.motor_error = False
        lizard_code = remove_indentation(f'''
            l0 = ODriveMotor({can.name}, {config.left_back_can_address}{', 6' if config.odrive_version == 6 else ''})
            r0 = ODriveMotor({can.name}, {config.right_back_can_address}{', 6' if config.odrive_version == 6 else ''})
            l1 = ODriveMotor({can.name}, {config.left_front_can_address}{', 6' if config.odrive_version == 6 else ''})
            r1 = ODriveMotor({can.name}, {config.right_front_can_address}{', 6' if config.odrive_version == 6 else ''})
            l0.m_per_tick = {m_per_tick}
            r0.m_per_tick = {m_per_tick}
            l1.m_per_tick = {m_per_tick}
            r1.m_per_tick = {m_per_tick}
            l0.reversed = {'true' if config.is_left_reversed else 'false'}
            r0.reversed = {'true' if config.is_right_reversed else 'false'}
            l1.reversed = {'true' if config.is_left_reversed else 'false'}
            r1.reversed = {'true' if config.is_right_reversed else 'false'}
            {config.name} = ODriveWheels(l0, r0)
            {config.name}_front = ODriveWheels(l1, r1)
            {config.name}.width = {width}
            {config.name}_front.width = {width}
            {config.name}.shadow({config.name}_front)
        ''')
        core_message_fields = [f'{config.name}.linear_speed:3', f'{config.name}.angular_speed:3']
        if config.odrive_version == 6:
            core_message_fields.extend(['l0.motor_error_flag', 'r0.motor_error_flag',
                                       'l1.motor_error_flag', 'r1.motor_error_flag'])
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)
        self.estop = estop

    async def drive(self, linear: float, angular: float) -> None:
        await super().drive(linear, angular)
        if linear == 0.0:
            linear = -0.0
        if angular == 0.0:
            angular = -0.0  # TODO: Temp fix
        if not self.robot_brain.is_ready:
            self.log.warning('Robot brain not ready')
            return
        await self.robot_brain.send(f'{self.config.name}.speed({linear}, {angular})')

    async def reset_motors(self) -> None:
        if self.estop.active:
            return
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

    def handle_core_output(self, time: float, words: list[str]) -> None:
        velocity = rosys.geometry.Velocity(linear=float(words.pop(0)), angular=float(words.pop(0)), time=time)
        if abs(velocity.linear) <= self.MAX_VALID_LINEAR_VELOCITY:
            self.VELOCITY_MEASURED.emit([velocity])
        else:
            self.log.error('Velocity is too high: (%s, %s)', velocity.linear, velocity.angular)

        if self.config.odrive_version == 6:
            self.motor_error = any([self.l0_error, self.r0_error, self.l1_error, self.r1_error])
            self.l0_error = int(words.pop(0))
            if self.l0_error == 1 and not self.motor_error:
                rosys.notify('Left Back Motor Error', 'warning')
            self.r0_error = int(words.pop(0))
            if self.r0_error == 1 and not self.motor_error:
                rosys.notify('Right Back Motor Error', 'warning')
            self.l1_error = int(words.pop(0))
            if self.l1_error == 1 and not self.motor_error:
                rosys.notify('Left Front Motor Error', 'warning')
            self.r1_error = int(words.pop(0))
            if self.r1_error == 1 and not self.motor_error:
                rosys.notify('Right Front Motor Error', 'warning')
                self.motor_error = True


class WheelsSimulationWithAcceleration(rosys.hardware.WheelsSimulation):
    def __init__(self, width: float = 0.5, *, linear_acceleration: float = 2.0, linear_deceleration: float = 0.5) -> None:
        """Simulate differential drive wheels with acceleration and deceleration handling.

        :param width: The distance between the wheels in meters.
        :param linear_acceleration: The maximum linear acceleration rate in m/s².
        :param linear_deceleration: The maximum linear deceleration rate in m/s².
        """
        super().__init__(width)

        self.linear_acceleration: float = linear_acceleration
        """The maximum linear acceleration rate."""

        self.linear_deceleration: float = linear_deceleration
        """The maximum linear deceleration rate."""

    @property
    def angular_acceleration(self) -> float:
        """Calculate angular acceleration from linear acceleration using differential drive kinematics."""
        return 2 * self.linear_acceleration / self.width

    @property
    def angular_deceleration(self) -> float:
        """Calculate angular deceleration from linear deceleration using differential drive kinematics."""
        return 2 * self.linear_deceleration / self.width

    async def drive(self, linear: float, angular: float) -> None:
        self.linear_target_speed = linear
        self.angular_target_speed = angular

    async def step(self, dt: float) -> None:
        if self.is_blocking:
            self.linear_velocity = 0
            self.angular_velocity = 0
            self.linear_target_speed = 0
            self.angular_target_speed = 0
        else:
            if self.linear_velocity < self.linear_target_speed:
                self.linear_velocity = min(self.linear_velocity + self.linear_acceleration * dt,
                                           self.linear_target_speed)
            elif self.linear_velocity > self.linear_target_speed:
                self.linear_velocity = max(self.linear_velocity - self.linear_deceleration * dt,
                                           self.linear_target_speed)
            if self.angular_velocity < self.angular_target_speed:
                self.angular_velocity = min(self.angular_velocity + self.angular_acceleration * dt,
                                            self.angular_target_speed)
            elif self.angular_velocity > self.angular_target_speed:
                self.angular_velocity = max(self.angular_velocity - self.angular_deceleration * dt,
                                            self.angular_target_speed)

        self.linear_velocity *= 1 - self.friction_factor
        self.angular_velocity *= 1 - self.friction_factor
        left_speed = self.linear_velocity - self.angular_velocity * self.width / 2
        right_speed = self.linear_velocity + self.angular_velocity * self.width / 2
        left_speed *= 1 - self.slip_factor_left
        right_speed *= 1 - self.slip_factor_right
        self.pose += rosys.geometry.PoseStep(linear=dt * (left_speed + right_speed) / 2,
                                             angular=dt * (right_speed - left_speed) / self.width,
                                             time=rosys.time())
        velocity = rosys.geometry.Velocity(linear=self.linear_velocity,
                                           angular=self.angular_velocity, time=self.pose.time)
        self.VELOCITY_MEASURED.emit([velocity])
