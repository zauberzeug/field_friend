import rosys
from rosys.helpers import remove_indentation


class LedEyesHardware(rosys.hardware.ModuleHardware):
    """Controls the LED eyes of the Field Friend robot."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain,
                 expander: rosys.hardware.ExpanderHardware | None,
                 name: str = 'led_eyes',
                 eyes_pin: int = 25,
                 ) -> None:
        self.name = name
        self.expander = expander
        self.eyes_pin = eyes_pin
        self.is_active: bool = False
        lizard_code = remove_indentation(f'''
            {name} = {expander.name + "." if expander else ""}Output({eyes_pin})
            {name}.on()
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)

    async def turn_on(self) -> None:
        await self.robot_brain.send(f'{self.name}.on()')
        self.is_active = True

    async def turn_off(self) -> None:
        await self.robot_brain.send(f'{self.name}.off()')
        self.is_active = False
