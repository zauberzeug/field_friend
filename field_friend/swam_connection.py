from __future__ import annotations

import logging
from typing import TYPE_CHECKING, cast

import requests
import rosys
from rosys.version import __version__ as rosys_version

from .automations.navigation import FieldNavigation
from .hardware import FieldFriendHardware

if TYPE_CHECKING:
    from .system import System


class SwarmConnection:
    def __init__(self, system: System, swarm_url: str, passphrase: str | None = None) -> None:
        self.log = logging.getLogger('field_friend.swarm_connection')
        self.system = system
        self.swarm_url = swarm_url
        self.passphrase = passphrase

        self.field_friend = system.field_friend
        self.automator = system.automator
        self.gnss = system.gnss
        rosys.on_repeat(self.send_status, 60)

    async def send_status(self):
        try:
            status = 'emergency stop' if len(self.field_friend.estop.pressed_estops) > 0 or self.field_friend.estop.is_soft_estop_active else \
                'bumper active' if self.field_friend.bumper and self.field_friend.bumper.active_bumpers else \
                'working' if self.automator.automation is not None and self.automator.automation.is_running else \
                'idle'
            if self.system.is_real:
                lizard_firmware = cast(FieldFriendHardware, self.field_friend).robot_brain.lizard_firmware
                await lizard_firmware.read_core_version()
                await lizard_firmware.read_p0_version()
                core_version = lizard_firmware.core_version
                p0_version = lizard_firmware.p0_version
            else:
                core_version = 'simulation'
                p0_version = 'simulation'
            if self.system.current_navigation is not None and self.system.current_navigation is FieldNavigation:
                field = self.system.field_navigation.field if self.system.field_navigation.field is not None else None
                row = self.system.field_navigation.current_row if self.system.field_navigation.current_row is not None else None
            else:
                field = None
                row = None
            position = self.gnss.current.location if self.gnss.current is not None else None
            data = {
                'version': self.system.version,
                'battery': self.field_friend.bms.state.percentage,
                'battery_charging': self.field_friend.bms.state.is_charging,
                'status': status,
                'position': {'lat': position.lat, 'long': position.long} if position is not None else None,
                # TODO: update the gnss quality with kalman filter
                'gnss_quality': self.gnss.current.gps_qual if self.gnss.current is not None else None,
                'implement': self.field_friend.implement_name,
                'navigation': self.system.current_navigation.name if self.system.current_navigation is not None else None,
                'field': field,
                'row': row,
                'rosys_version': rosys_version,
                'core_lizard_version': core_version,
                'p0_lizard_version': p0_version,
            }
            endpoint = f'{self.swarm_url}/api/robot/{self.system.robot_id.lower()}'
            headers = {'passphrase': self.passphrase} if self.passphrase is not None else {}
            response = requests.post(endpoint, json=data, headers=headers, timeout=5)
            if response.status_code != 200:
                rosys.notify(f'Response code {response.status_code}.', type='negative')
        except Exception as e:
            rosys.notify(f'Error sending status: {e!s}', type='negative')
