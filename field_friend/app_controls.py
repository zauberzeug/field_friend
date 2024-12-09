import rosys
from rosys.automation import Automator
from rosys.automation.app_controls_ import AppControls as RosysAppControls
from rosys.hardware import RobotBrain

from .hardware.field_friend import FieldFriend


class AppControls(RosysAppControls):

    def __init__(self,
                 robot_brain: RobotBrain,
                 automator: Automator,
                 field_friend: FieldFriend,
                 ) -> None:
        super().__init__(robot_brain, automator)
        self.field_friend = field_friend
        self.last_battery_percentage: float | None = self.field_friend.bms.state.percentage
        self.last_charging: bool | None = self.field_friend.bms.state.is_charging
        self.last_info: str = ''
        self.APP_CONNECTED.register(self.reset)
        if self.field_friend.bumper:
            self.field_friend.bumper.BUMPER_TRIGGERED.register(self.sync)
        self.field_friend.estop.ESTOP_TRIGGERED.register(self.sync)
        rosys.on_repeat(self.check_battery, 5.0)

    async def check_battery(self) -> None:
        if self.field_friend.bms.state.percentage != self.last_battery_percentage or self.field_friend.bms.state.is_charging != self.last_charging:
            self.last_battery_percentage = self.field_friend.bms.state.percentage
            self.last_charging = self.field_friend.bms.state.is_charging
            await self.sync()

    async def sync(self) -> None:
        await super().sync()
        infos = []
        if self.field_friend.estop.active:
            infos.append('E Stop active')
        if self.field_friend.bumper and self.field_friend.bumper.active_bumpers:
            infos.append(f'Bumper ({" ".join(self.field_friend.bumper.active_bumpers)}) active')
        if self.field_friend.bms.state.is_charging:
            infos.append('charging battery')
        battery = f'{self.field_friend.bms.state.percentage:.0f}%' if self.field_friend.bms.state.percentage is not None else '?'
        infos.append(f'Battery: {battery}')

        info = ' | '.join(infos)
        if info != self.last_info:
            await self.set_info(info)
            self.last_info = info

    def reset(self) -> None:
        self.last_info = 'loading'
