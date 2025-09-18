from typing import cast

import rosys
from nicegui import app

from field_friend.hardware import FieldFriendHardware
from field_friend.system import System


class Status:
    """API endpoints for retrieving the robot's status information.

    This API provides endpoints to check the robot's operational status, including
    work status, battery level, position, and software versions.
    """

    def __init__(self, system: System) -> None:
        self.system = system

        @app.get('/api/status')
        async def status():
            work_status = 'emergency stop' if len(self.system.field_friend.estop.pressed_estops) > 0 or self.system.field_friend.estop.is_soft_estop_active else \
                'bumper active' if self.system.field_friend.bumper and self.system.field_friend.bumper.active_bumpers else \
                'working' if self.system.automator.automation is not None and self.system.automator.automation.is_running else \
                'paused' if self.system.automator.automation is not None and self.system.automator.automation.is_paused else \
                'idle'
            if rosys.is_simulation():
                core_version = 'simulation'
                p0_version = 'simulation'
            else:
                try:
                    lizard_firmware = cast(FieldFriendHardware, self.system.field_friend).robot_brain.lizard_firmware
                    await lizard_firmware.read_core_version()
                    await lizard_firmware.read_p0_version()
                    core_version = lizard_firmware.core_version or 'unknown'
                    p0_version = lizard_firmware.p0_version or 'unknown'
                except rosys.hardware.robot_brain.EspNotReadyException:
                    core_version = 'unknown'
                    p0_version = 'unknown'

            if hasattr(self.system, 'field_navigation') and self.system.field_navigation is not None and (self.system.automator.is_running or self.system.automator.is_paused):
                field = self.system.field_navigation.field.name if self.system.field_navigation is not None and self.system.field_navigation.field else None
                row = self.system.field_navigation.current_row.name if self.system.field_navigation is not None and self.system.field_navigation.current_row else None
            else:
                field = None
                row = None
            position = self.system.gnss.last_measurement.point.degree_tuple if self.system.gnss is not None and self.system.gnss.last_measurement is not None else None
            data = {
                'robot_id': self.system.robot_id,
                'brain_id': self.system.config.robot_brain.name,
                'battery': self.system.field_friend.bms.state.percentage,
                'battery_charging': self.system.field_friend.bms.state.is_charging,
                'status': work_status,
                'position': {'lat': position[0], 'lon': position[1]} if position is not None else None,
                # TODO: update the gnss quality with kalman filter
                'gnss_quality': self.system.gnss.last_measurement.gps_quality if self.system.gnss is not None and self.system.gnss.last_measurement is not None else None,
                'implement': self.system.field_friend.implement_name,
                'navigation': self.system.current_navigation.name if self.system.current_navigation is not None else None,
                'field': field,
                'row': row,
                'rosys_version': rosys.__version__,
                'core_lizard_version': core_version,
                'p0_lizard_version': p0_version,
            }
            return data
