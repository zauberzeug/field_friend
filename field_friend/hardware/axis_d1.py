import rosys
from rosys.analysis import track
from rosys.automation import uninterruptible
from rosys.helpers import remove_indentation

from ..config import AxisD1Configuration
from .axis import Axis

D1_STEPS_P_M = 100000


class AxisD1(Axis, rosys.hardware.ModuleHardware):
    """Rosys module to control the Igus D1 motor controller."""

    def __init__(self, config: AxisD1Configuration, robot_brain: rosys.hardware.RobotBrain, *, can: rosys.hardware.CanHardware, **kwargs) -> None:
        self.config = config
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
            {config.name}_motor = D1Motor({can.name}, {config.can_address})
            {config.name}_motor.homing_acceleration = {config.homing_acceleration}
            {config.name}_motor.switch_search_speed = {config.homing_velocity}
            {config.name}_motor.zero_search_speed = {config.homing_velocity}
            {config.name}_motor.profile_acceleration = {config.profile_acceleration}
            {config.name}_motor.profile_deceleration = {config.profile_deceleration}
            {config.name}_motor.profile_velocity = {config.profile_velocity}
        ''')
        core_message_fields = [
            f'{config.name}_motor.position',
            f'{config.name}_motor.velocity',
            f'{config.name}_motor.status_word',
            f'{config.name}_motor.status_flags',
        ]
        super().__init__(
            robot_brain=robot_brain,
            lizard_code=lizard_code,
            core_message_fields=core_message_fields,
            max_speed=config.profile_velocity,
            max_position=config.max_position,
            min_position=config.min_position,
            axis_offset=config.axis_offset,
            reference_speed=config.homing_velocity,
            steps_per_m=D1_STEPS_P_M,
            reversed_direction=config.reversed_direction,
            **kwargs)
        rosys.on_startup(self.set_speed_parameters)

    async def stop(self):
        pass

    @track
    @uninterruptible
    async def move_to(self, position: float, speed: int | None = None) -> None:
        await super().move_to(position)
        if not self._valid_status():
            await self.enable_motor()
        while (abs(self.position - position)) > 0.01:
            # TODO: add timeout and error message
            # sometimes the moving command is not executed, so it is send in each loop (for demo purposes)
            await self.robot_brain.send(f'{self.config.name}_motor.profile_position({self.compute_steps(position)});')
            await rosys.sleep(0.1)

    @track
    @uninterruptible
    async def set_speed_parameters(self):
        await self.robot_brain.send(f'{self.config.name}_motor.homing_acceleration = {self.config.homing_acceleration};')
        await self.robot_brain.send(f'{self.config.name}_motor.switch_search_speed = {self.config.homing_velocity};')
        await self.robot_brain.send(f'{self.config.name}_motor.zero_search_speed = {self.config.homing_velocity};')
        await self.robot_brain.send(f'{self.config.name}_motor.profile_acceleration = {self.config.profile_acceleration};')
        await self.robot_brain.send(f'{self.config.name}_motor.profile_deceleration = {self.config.profile_deceleration};')

    @track
    @uninterruptible
    async def enable_motor(self):
        if self.fault:
            await self.reset_fault()
        self.log.debug(f'AxisD1 {self.config.name} motor.setup()')
        await self.robot_brain.send(f'{self.config.name}_motor.setup()')
        await rosys.sleep(2.0)
        self.log.debug(f'AxisD1 {self.config.name} motor.setup() done')

    @track
    @uninterruptible
    async def reset_fault(self):
        if self.fault:
            self.log.debug(f'AxisD1 {self.config.name} motor.reset()')
            await self.robot_brain.send(f'{self.config.name}_motor.reset()')
            await rosys.sleep(0.5)
            self.log.debug(f'AxisD1 {self.config.name} motor.reset() done')
        else:
            self.log.error(f'AxisD1 {self.config.name} is not in fault state')

    @track
    @uninterruptible
    async def try_reference(self) -> bool:
        if not self._valid_status():
            await self.enable_motor()
            self._valid_status()
        if self.is_referenced:
            self.log.error(f'AxisD1 {self.config.name} is already referenced')
        else:
            # due to some timing issues, the homing command is sent twice
            await self.robot_brain.send(f'{self.config.name}_motor.home()')
            await self.robot_brain.send(f'{self.config.name}_motor.home()')
            while not self.is_referenced:
                await rosys.sleep(0.1)
        return self.is_referenced

    @track
    async def recover(self):
        await self.reset_fault()
        await self.try_reference()

    @track
    @uninterruptible
    async def speed_mode(self, speed: int):
        if not self._valid_status():
            await self.enable_motor()
        # due to some timing issues, the speed command is sent twice
        await self.robot_brain.send(f'{self.config.name}_motor.profile_velocity({speed});')
        await self.robot_brain.send(f'{self.config.name}_motor.profile_velocity({speed});')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.steps = int(words.pop(0))
        self.velocity = int(words.pop(0))
        self.statusword = int(words.pop(0))
        self.is_referenced = int(words.pop(0)) == 1
        self._split_statusword()

    def _valid_status(self) -> bool:
        is_valid = self.ready_to_switch_on and self.switched_on and self.operation_enabled and self.quick_stop
        self.log.debug(f'''AxisD1 {self.config.name} status: {is_valid}
                            ready_to_switch_on: {self.ready_to_switch_on}
                            switched_on: {self.switched_on}
                            operation_enabled: {self.operation_enabled}
                            quick_stop: {self.quick_stop}
                            ''')
        return is_valid

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
