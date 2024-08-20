from typing import Optional

import rosys
from rosys.helpers import remove_indentation

from .y_axis import YAxis
from .z_axis import ZAxis


class D1Axis(YAxis, ZAxis, rosys.hardware.module.ModuleHardware):
    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 max_speed: int = 2000,
                 reference_speed: int = 40,
                 name: str = 'axis_D1',
                 can: rosys.hardware.CanHardware,
                 can_address: int = 0x60,
                 min_position: float = -0.068,
                 max_position: float = 0.068,
                 axis_offset: float = 0.075,
                 steps_per_m: float = 1_481_481.48,  # [steps/turn] / ([gear] * [m/turn])
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
        self.statusword = 0
        self.status_flag = 0
        self.poition = 0
        self.velocity = 0
        lizard_code = remove_indentation(f'''
            {name}_motor = D1Motor({can.name}, {can_address})
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
        # invalidate the state, to stop motor
        await self.robot_brain.send(f'{self.name}_motor.sdo_write(0x6040, 0, 16, 6)')

    async def move_to(self, position: float, speed: float) -> None:
        if self.statusword == 5671 and self.status_flag == 1:
            await self.robot_brain.send(f'{self.name}_motor.ppMode({position})')
        if self.status_flag == 0:
            self.log.error(f'd1axis {self.name} is not refernced')
        if self.statusword != 5671:
            self.log.error(f'd1axis {self.name} is not in correct state')

    async def enable_motor(self):
        await self.robot_brain.send(f'{self.name}_motor.setup()')

    async def reset_error(self):
        await self.robot_brain.send(f'{self.name}_motor.reset()')

    async def try_reference(self):
        if self.status_flag == 1:
            self.log.error(f'd1axis {self.name} is already referenced')
        await self.robot_brain.send(f'{self.name}_motor.homing()')

    async def speed_Mode(self, speed: int):
        await self.robot_brain.send(f'{self.name}_motor.speedMode({speed})')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        self.position = words.pop(0)
        self.velocity = words.pop(0)
        self.statusword = words.pop(0)
        self.status_flag = words.pop(0)
        if self.status_flag == 1:
            self.is_referenced = True
        else:
            self.is_referenced = False
