import logging

import rosys

from ..hardware import FieldFriendHardware


class BatteryWatcher:
    def __init__(self,
                 field_friend: FieldFriendHardware,
                 automator: rosys.automation.Automator,
                 ) -> None:
        self.field_friend = field_friend
        self.automator = automator
        self.log = logging.getLogger('field_friend.battery_watcher')
        self.was_charging = False
        rosys.on_startup(self.release_relais_on_startup)
        rosys.on_repeat(self.check_battery, 0.5)

    async def check_battery(self) -> None:
        if self.automator.is_running or self.field_friend.wheels.linear_target_speed != 0.0:
            return
        if self.field_friend.bms.state.is_charging:
            self.was_charging = True
            return
        if not self.field_friend.bms.state.is_charging and self.was_charging:
            self.log.info('battery charging-state change detected...')
            await self.field_friend.battery_control.release_battery_relay()
            self.was_charging = False

    async def release_relais_on_startup(self) -> None:
        self.log.info('releasing battery relay on rosys startup')
        await self.field_friend.battery_control.release_battery_relay()
