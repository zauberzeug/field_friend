from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .axis import Axis


class D1Axis(Axis, rosys.hardware.ModuleHardware):
    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 max_speed: int = 2000,
                 reference_speed: int = 40,
                 name: str = 'axis_D1',
                 can: rosys.hardware.CanHardware,
                 can_address: int = 0x60,
                 min_position: float = 1,
                 max_position: float = 100,
                 axis_offset: float = 0,
                 steps_per_m: float = 1,  # [steps/turn] / ([gear] * [m/turn])
                 reversed_direction: bool = False,
                 ) -> None:
        """
        Initialize the D1Axis object.

        Args:
            robot_brain (rosys.hardware.RobotBrain): The RobotBrain object.
            max_speed (int, optional): The maximum speed of the axis. Defaults to 2000.
            reference_speed (int, optional): The reference speed of the axis. Defaults to 40.
            name (str, optional): The name of the axis. Defaults to 'axis_D1'.
            can (rosys.hardware.CanHardware): The CAN hardware object.
            can_address (int, optional): The CAN address of the axis. Defaults to 0x60.
            min_position (float, optional): The minimum position of the axis. Defaults to -0.068.
            max_position (float, optional): The maximum position of the axis. Defaults to 0.068.
            axis_offset (float, optional): The offset of the axis. Defaults to 0.075.
            steps_per_m (float, optional): The steps per meter of the axis. Defaults to 1_481_481.48.
            reversed_direction (bool, optional): Whether the direction of the axis is reversed. Defaults to False.
        """
        self.name = name
        self.statusword: int = 0
        # actual position of the motor
        self.actual_position: int = 0
        # actuela current velocity of the motor
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
        ''')
        core_message_fields = [
            f'{name}_motor.position',
            f'{name}_motor.velocity',
            f'{name}_motor.statusword',
            f'{name}_motor.status_flag',
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

    async def stop(self):
        await self.speed_Mode(0)

    async def move_to(self, newposition: float) -> None:
        if self.is_referenced:
            await self.robot_brain.send(f'{self.name}_motor.ppMode({newposition});')
        if not self.is_referenced:
            self.log.error(f'd1axis {self.name} is not refernced')

    def valid_status(self) -> bool:
        return self.ready_to_switch_on and self.switched_on and self.operation_enabled and self.quick_stop

    async def enable_motor(self):
        if self.fault:
            await self.reset_error()
            rosys.sleep(0.5)
        await self.robot_brain.send(f'{self.name}_motor.setup()')

    async def reset_error(self):
        if self.fault:
            await self.robot_brain.send(f'{self.name}_motor.reset()')
        self.log.error(f'd1axis {self.name} is not in fault state')

    async def try_reference(self):
        if not self.valid_status():
            await self.enable_motor()
        if self.is_referenced:
            self.log.error(f'd1axis {self.name} is already referenced')
        else:
            await self.robot_brain.send(f'{self.name}_motor.homing()')
            await self.robot_brain.send(f'{self.name}_motor.homing()')

    async def speed_Mode(self, speed: int):

        await self.robot_brain.send(f'{self.name}_motor.speedMode({speed});')
        # else:
        #     self.log.error(f'd1axis {self.name} is not in correct state')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.actual_position = words.pop(0)
        self.velocity = words.pop(0)
        self.statusword = int(words.pop(0))
        self.is_referenced = int(words.pop(0)) == 1
        self.split_statusword()

    def split_statusword(self) -> None:
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
