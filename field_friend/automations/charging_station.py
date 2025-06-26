from __future__ import annotations

from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.geometry import Pose

if TYPE_CHECKING:
    from field_friend.system import System


class ChargingStation:
    DOCKING_DISTANCE = 2.0
    DOCKING_SPEED = 0.1

    def __init__(self, system: System):
        self.system = system
        self.docking_distance = self.DOCKING_DISTANCE
        self.docking_speed = self.DOCKING_SPEED

    async def approach(self):
        rosys.notify('Approaching not implemented yet')

    async def dock(self):
        rosys.notify('Docking to charging station')

        async def move():
            robot_pose = self.system.robot_locator.pose
            with self.system.driver.parameters.set(can_drive_backwards=True, linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_to(robot_pose.transform_pose(Pose(x=-self.docking_distance, y=0)).point, backward=True)
        self.system.automator.start(move())

    async def undock(self):
        rosys.notify('Detaching from charging station')

        async def move():
            robot_pose = self.system.robot_locator.pose
            with self.system.driver.parameters.set(linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_to(robot_pose.transform_pose(Pose(x=self.docking_distance, y=0)).point)
        self.system.automator.start(move())

    def developer_ui(self):
        with ui.column():
            ui.label('Charging Station').classes('text-center text-bold')
            ui.number(label='Docking distance', min=0, step=0.01, format='%.3f', suffix='m', value=self.docking_distance) \
                .classes('w-4/5').bind_value_to(self, 'docking_distance')
            ui.number(label='Docking speed', min=0, step=0.01, format='%.2f', suffix='m/s', value=self.docking_speed) \
                .classes('w-4/5').bind_value_to(self, 'docking_speed')
            ui.button('Dock', on_click=self.dock)
            ui.button('Undock', on_click=self.undock)
            ui.label('Charging').bind_text_from(self.system.field_friend.bms.state, 'is_charging',
                                                lambda is_charging: 'Charging' if is_charging else 'Not charging')
