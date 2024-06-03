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
        rosys.on_repeat(self.restart_core, 10)

    def _on_error(self, line: str) -> None:
        if 'twai' in line and not self.error:
            # TODO: find a better to fix it
            self.error = True

    async def restart_core(self) -> None:
        if self.error:
            await self.robot_brain.send('core.restart();')
            await rosys.sleep(5)
            self.error = False
