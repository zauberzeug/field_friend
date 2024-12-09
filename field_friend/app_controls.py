import rosys
from rosys.automation import Automator
from rosys.automation import app_controls as RosysAppControls  # noqa ignore this due to being a class
from rosys.hardware import RobotBrain

from .hardware.field_friend import FieldFriend


class AppControls(RosysAppControls):

    def __init__(self,
                 robot_brain: RobotBrain,
                 automator: Automator,
                 robot: FieldFriend,
                 ) -> None:
        super().__init__(robot_brain, automator)
        self.robot = robot
        self.last_battery_percentage: float | None = self.robot.bms.state.percentage
        self.last_charging: bool | None = self.robot.bms.state.is_charging
        self.last_info: str = ''
        self.APP_CONNECTED.register(self.reset)
        if self.robot.bumper:
            self.robot.bumper.BUMPER_TRIGGERED.register(self.sync)
        self.robot.estop.ESTOP_TRIGGERED.register(self.sync)
        rosys.on_repeat(self.check_battery, 5.0)

    async def check_battery(self) -> None:
        if self.robot.bms.state.percentage != self.last_battery_percentage or self.robot.bms.state.is_charging != self.last_charging:
            self.last_battery_percentage = self.robot.bms.state.percentage
            self.last_charging = self.robot.bms.state.is_charging
            await self.sync()

    async def sync(self) -> None:
        await super().sync()
        infos = []

        if self.robot.estop.active:
            infos.append('E Stop active')
        if self.robot.bumper and self.robot.bumper.active_bumpers:
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
