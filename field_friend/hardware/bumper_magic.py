import abc

import rosys
from rosys.helpers import remove_indentation


class BumperMagic(rosys.hardware.Module, abc.ABC):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.BUMPER_MAGIC_CHANGED = rosys.event.Event()
        """Event that is emitted whenever the bumper magic state changes. (argument: active: bool)"""

        self.active: bool = False

    async def disable(self) -> None:
        self.active = False


class BumperMagicHardware(BumperMagic, rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 bumper_front_name: str = 'bumper_front_top',
                 ) -> None:

        lizard_code = remove_indentation(f'''
            int bump_code = 0
            int first_bump_time = 0
            when bump_code == 4 and {bumper_front_name}.change > 0 then bump_code = 0; end
            when bump_code == 3 and {bumper_front_name}.change  > 0 then bump_code = 4; end
            when bump_code == 2 and {bumper_front_name}.change  > 0 then bump_code = 3; end
            when bump_code == 1 and {bumper_front_name}.change  > 0 then bump_code = 2; end
            when bump_code == 0 and {bumper_front_name}.change  > 0 then bump_code = 1; first_bump_time = core.millis; end
            when bump_code != 4 and core.millis - first_bump_time > 5000 then bump_code = 0; end
        ''')
        core_message_fields = ['bump_code']
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code, core_message_fields=core_message_fields)

    async def disable(self) -> None:
        await super().disable()
        await self.robot_brain.send('bump_code = 0;')

    def handle_core_output(self, time: float, words: list[str]) -> None:
        active = int(words.pop(0)) == 4
        if active != self.active:
            self.active = active
            self.BUMPER_MAGIC_CHANGED.emit(self.active)


class BumperMagicSimulation(BumperMagic, rosys.hardware.ModuleSimulation):
    pass
