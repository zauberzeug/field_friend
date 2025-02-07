import rosys
from rosys.helpers import remove_indentation

from .axis import Axis

D1_STEPS_P_M = 100000


class AxisD1(Axis, rosys.hardware.ModuleHardware):
    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 name: str = 'axis_D1',
                 can: rosys.hardware.CanHardware,
                 max_position: float = -0.1,
                 min_position: float = 0.1,
                 axis_offset: int = 0,
                 can_address: int = 0x60,
                 homing_acceleration: int = 100,
                 homing_velocity: int = 20,
                 profile_velocity: int = 20,
                 profile_acceleration: int = 200,
                 profile_deceleration: int = 400,
                 reverse_direction: bool = False,


                 ** kwargs
                 ) -> None:
        """Rosys module to control the Igus D1 motor controller.

        :param: robot_brain: The RobotBrain object.
        :param name: The name of the axis (default: 'axis_D1').
        :param can: The CAN hardware object.
        :param can_address: The CAN address of the axis (default: 0x60).
        """
        self.name = name
        self.statusword: int = 0
        self.steps: int = 0
        self.velocity: int = 0

        # flags of the Statusword for more information refer to the CANopen standard and D1 manual
        self.ready_to_switch_on: bool = False
        self.switched_on: bool = False
        self.operation_enabled: bool = False
        self.fault: bool = False
        self.voltage_enabled: bool = False
        self.quick_stop: bool = False
        self.switch_on_disabled: bool = False
        self.warning: bool = False
        self.manufacturer_specific: bool = False  # this has no funktion in the D1
        self.remote_enable: bool = False
        self.target_reached: bool = False
        self.internal_limit_active: bool = False
        self.operation_mode_specific: bool = False
        self.manufacturer_specific2: bool = False

        lizard_code = remove_indentation(f'''
            {self.name}_motor = D1Motor({can.name}, {can_address})
            {self.name}_motor.homing_acceleration = {homing_acceleration}
            {self.name}_motor.switch_search_speed = {homing_velocity}
            {self.name}_motor.zero_search_speed = {homing_velocity}
            {self.name}_motor.profile_acceleration = {profile_acceleration}
            {self.name}_motor.profile_deceleration = {profile_deceleration}
            {self.name}_motor.profile_velocity = {profile_velocity}
        ''')
        core_message_fields = [
            f'{name}_motor.position',
            f'{name}_motor.velocity',
            f'{name}_motor.status_word',
            f'{name}_motor.status_flags',
        ]
        super().__init__(
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields,
            max_speed=profile_velocity,
            max_position=max_position,
            min_position=min_position,
            axis_offset=axis_offset,
            reference_speed=homing_velocity,
            steps_per_m=D1_STEPS_P_M,
            reversed_direction=reverse_direction,
            **kwargs)

    async def stop(self):
        pass

    async def move_to(self, position: float, speed: int | None = None) -> None:
        await super().move_to(position)
        if not self._valid_status():
            await self.enable_motor()
        while (abs(self.position - position)) > 0.01:
            # sometimes the moving command is not executed, so it is send in each loop (for demo purposes)
            await self.robot_brain.send(f'{self.name}_motor.profile_position({self.compute_steps(position)});')
            await rosys.sleep(0.1)

    async def enable_motor(self):
        if self.fault:
            await self.reset_error()
            await rosys.sleep(0.5)
        await self.robot_brain.send(f'{self.name}_motor.setup()')

    async def reset_error(self):
        if self.fault:
            await self.robot_brain.send(f'{self.name}_motor.reset()')
        else:
            self.log.error(f'AxisD1 {self.name} is not in fault state')

    async def try_reference(self) -> bool:
        if not self._valid_status():
            await self.enable_motor()
        if self.is_referenced:
            self.log.error(f'AxisD1 {self.name} is already referenced')
        else:
            # due to some timing issues, the homing command is sent twice
            await self.robot_brain.send(f'{self.name}_motor.home()')
            await self.robot_brain.send(f'{self.name}_motor.home()')
            while not self.is_referenced:
                await rosys.sleep(0.1)
        return self.is_referenced

    async def speed_mode(self, speed: int):
        if not self._valid_status():
            await self.enable_motor()
        # due to some timing issues, the speed command is sent twice
        await self.robot_brain.send(f'{self.name}_motor.profile_velocity({speed});')
        await self.robot_brain.send(f'{self.name}_motor.profile_velocity({speed});')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.steps = int(words.pop(0))
        self.velocity = int(words.pop(0))
        self.statusword = int(words.pop(0))
        self.is_referenced = int(words.pop(0)) == 1
        self._split_statusword()

    def _valid_status(self) -> bool:
        return self.ready_to_switch_on and self.switched_on and self.operation_enabled and self.quick_stop

    def _split_statusword(self) -> None:
        self.ready_to_switch_on = ((self.statusword >> 0) & 1) == 1
        self.switched_on = ((self.statusword >> 1) & 1) == 1
        self.operation_enabled = ((self.statusword >> 2) & 1) == 1
        self.fault = ((self.statusword >> 3) & 1) == 1
        self.voltage_enabled = ((self.statusword >> 4) & 1) == 1
        self.quick_stop = ((self.statusword >> 5) & 1) == 1
        self.switch_on_disabled = ((self.statusword >> 6) & 1) == 1
        self.warning = ((self.statusword >> 7) & 1) == 1
        self.manufacturer_specific = ((self.statusword >> 8) & 1) == 1  # No function in D1
        self.remote_enable = ((self.statusword >> 9) & 1) == 1
        self.target_reached = ((self.statusword >> 10) & 1) == 1
        self.internal_limit_active = ((self.statusword >> 11) & 1) == 1
        self.operation_mode_specific = ((self.statusword >> 12) & 1) == 1
        self.manufacturer_specific2 = ((self.statusword >> 13) & 1) == 1
