import rosys
from rosys.helpers import remove_indentation


class CanOpenMasterHardware(rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 name: str = 'master',
                 sync_interval: int = 5
                 ) -> None:
        self.name = name
        self.error = False
        lizard_code = remove_indentation(f'''
            {name} = CanOpenMaster({can.name})
            {name}.sync_interval = {sync_interval}
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
        self.message_hooks['error'] = self._on_error

    async def _on_error(self, line: str) -> None:
        if 'twai' in line and not self.error:
            # TODO: find a better to fix it
            self.error = True
            await self.robot_brain.send('core.restart();')
            await rosys.sleep(5.0)
            self.error = False
