import rosys
from rosys.helpers import remove_indentation


class CanOpenMasterHardware(rosys.hardware.ModuleHardware):

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 can: rosys.hardware.CanHardware,
                 name: str = 'master',
                 sync_interval: int = 5
                 ) -> None:
        self.name = name
        lizard_code = remove_indentation(f'''
            {name} = CanOpenMaster({can.name})
            {name}.sync_interval = {sync_interval}
        ''')
        super().__init__(robot_brain=robot_brain, lizard_code=lizard_code)
