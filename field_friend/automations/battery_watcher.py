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

        if self.field_friend.battery_control:
            rosys.on_startup(self._release_relais_on_startup)
            rosys.on_repeat(self._check_battery, 0.5)
        else:
            self.log.warning('no battery control hardware found, battery watcher will not be active')

    async def _check_battery(self) -> None:
        if not self.field_friend.battery_control:
            return

        if self.automator.is_running or self.field_friend.wheels.linear_target_speed != 0.0:
            return
        if self.field_friend.bms.state.is_charging:
            self.was_charging = True
            return
        if not self.field_friend.bms.state.is_charging and self.was_charging:
            self.log.info('battery charging-state change detected...')
            await self.field_friend.battery_control.release_battery_relay()
            self.was_charging = False

    async def _release_relais_on_startup(self) -> None:
        if not self.field_friend.battery_control:
            return

        self.log.info('releasing battery relay on rosys startup')
        await rosys.sleep(15)
        await self.field_friend.battery_control.release_battery_relay()
