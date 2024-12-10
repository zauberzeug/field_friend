import logging

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
        self.last_estops_pressed: list[int] = []
        self.last_estop_active: bool = self.field_friend.estop.active
        self.last_estop_soft_active: bool = self.field_friend.estop.is_soft_estop_active
        self.last_bumpers_active: list[str] = []
        self.last_info: str = ''
        self.APP_CONNECTED.register(self.reset)
        rosys.on_repeat(self.check_status, 2.0)

    async def check_status(self) -> None:
        logging.warning("checking status")
        logging.warning(self.field_friend.estop.pressed_estops)
        logging.warning(self.last_estops_pressed)
        estop_changed = self.field_friend.estop.active != self.last_estop_active or self.field_friend.estop.pressed_estops != self.last_estops_pressed or self.field_friend.estop.is_soft_estop_active != self.last_estop_soft_active
        if estop_changed:
            self.last_estop_active = self.field_friend.estop.active
            self.last_estops_pressed = list(self.field_friend.estop.pressed_estops)
            self.last_estop_soft_active = self.field_friend.estop.is_soft_estop_active
        battery_changed = self.field_friend.bms.state.percentage != self.last_battery_percentage or self.field_friend.bms.state.is_charging != self.last_charging
        if battery_changed:
            self.last_battery_percentage = self.field_friend.bms.state.percentage
            self.last_charging = self.field_friend.bms.state.is_charging
        bumpers_changed = self.last_bumpers_active != self.field_friend.bumper.active_bumpers if self.field_friend.bumper else False
        if bumpers_changed:
            assert self.field_friend.bumper is not None
            self.last_bumpers_active = self.field_friend.bumper.active_bumpers
        print(estop_changed, battery_changed, bumpers_changed)
        if estop_changed or battery_changed or bumpers_changed:
            await self.sync()

    async def sync(self) -> None:
        await super().sync()
        infos = []
        if self.field_friend.estop.active:
            infos.append(
                f'E-Stop {" and ".join(str(estop) for estop in self.field_friend.estop.pressed_estops)} active. Please check the robot.')
        elif self.field_friend.estop.is_soft_estop_active:
            infos.append('Soft E-Stop active. Please check the robot.')
        elif self.field_friend.bumper and self.field_friend.bumper.active_bumpers:
            infos.append('A bumper is active. Please check the robot.')
        else:
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
