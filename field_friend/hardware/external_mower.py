

import rosys
from rosys.helpers import remove_indentation


class MowerHardware(rosys.hardware.ModuleHardware):
    """This module implements extrernal mower hardware.

    on and off commands are forwarded to a given Robot Brain.
    """

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 name: str = 'mower',
                 m0_can_address: int = 0x000,
                 m1_can_address: int = 0x100,
                 m2_can_address: int = 0x200,
                 m_per_tick: float = 0.01,
                 speed: float = 0.0,
                 is_m0_reversed: bool = False,
                 is_m1_reversed: bool = False,
                 is_m2_reversed: bool = False,
                 odrive_version: int = 4,) -> None:
        self.name = name
        self.speed = speed
        self.m0_error = 0
        self.m1_error = 0
        self.m2_error = 0
        self.motor_error = False
        self.odrive_version = odrive_version
        lizard_code = remove_indentation(f'''
            m0 = ODriveMotor({can.name}, {m0_can_address})
            m1 = ODriveMotor({can.name}, {m1_can_address})
            m2 = ODriveMotor({can.name}, {m2_can_address})
            m0.m_per_tick = {m_per_tick}
            m1.m_per_tick = {m_per_tick}
            m2.m_per_tick = {m_per_tick}
            m0.reversed = {'true' if is_m0_reversed else 'false'}
            m1.reversed = {'true' if is_m1_reversed else 'false'}
            m2.reversed = {'true' if is_m2_reversed else 'false'}
            m0.shadow(m1)
            m0.shadow(m2)
        ''')
        core_message_fields = []
        if self.odrive_version == 6:
            core_message_fields.extend(['m0.motor_error_flag', 'm1.motor_error_flag',
                                       'm2.motor_error_flag'])
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def turn_on(self) -> None:
        await self.robot_brain.send(f'm0.speed({self.speed})')

    async def turn_off(self) -> None:
        await self.robot_brain.send('m0.off()')

    async def reset_motors(self) -> None:
        if not self.motor_error:
            return
        if self.m0_error == 1:
            await self.robot_brain.send('m0.reset_motor()')
        if self.m1_error == 1:
            await self.robot_brain.send('m1.reset_motor()')
        if self.m2_error == 1:
            await self.robot_brain.send('m2.reset_motor()')
        self.motor_error = False

    def handle_core_output(self, time: float, words: list[str]) -> None:
        if self.odrive_version == 6:
            self.m0_error = int(words.pop(0))
            if self.m0_error == 1:
                rosys.notify('warning' 'Left Back Motor Error')
                self.motor_error = True
            self.m1_error = int(words.pop(0))
            if self.m1_error == 1:
                rosys.notify('warning' 'Right Back Motor Error')
                self.motor_error = True
            self.m2_error = int(words.pop(0))
            if self.m2_error == 1:
                rosys.notify('warning' 'Left Front Motor Error')
                self.motor_error = True