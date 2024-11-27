from rosys.automation import Automator
from rosys.automation import app_controls as RosysAppControls
from rosys.hardware import RobotBrain
import rosys
from .hardware.field_friend import FieldFriend


class AppControls(RosysAppControls):

    def __init__(self,
                 robot_brain: RobotBrain,
                 automator: Automator,
                 robot: FieldFriend,
                 ) -> None:
        super().__init__(robot_brain, automator)
        self.robot = robot
        self.last_info: str = ''
        self.APP_CONNECTED.register(self.reset)
        rosys.on_repeat(self.sync, 1.0)

    async def sync(self) -> None:
        await super().sync()
        infos = []

        if self.robot.estop.active:
            infos.append("E Stop active")
        if self.robot.bumper.active_bumpers:
            infos.append(f'Bumper ({" ".join(self.robot.bumper.active_bumpers)}) active')
        if self.robot.bms.state.is_charging:
            infos.append('charging battery')
        battery = f'{self.robot.bms.state.percentage:.0f}%' if self.robot.bms.state.percentage is not None else '?'
        infos.append(f'Battery: {battery}')

        info = ' | '.join(infos)
        if info != self.last_info:
            await self.set_info(info)
            self.last_info = info

    def reset(self) -> None:
        self.last_info = 'loading'
