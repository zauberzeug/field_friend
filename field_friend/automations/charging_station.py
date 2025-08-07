from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.geometry import GeoPose, Spline
from rosys.hardware.gnss import GpsQuality

if TYPE_CHECKING:
    from field_friend.system import System


class ChargingStation:
    DOCKING_DISTANCE = 2.0
    DOCKING_SPEED = 0.1

    def __init__(self, system: System):
        self.log = logging.getLogger(__name__)
        self.system = system
        self.bms = system.field_friend.bms

        self.docking_distance = self.DOCKING_DISTANCE
        self.docking_speed = self.DOCKING_SPEED
        # TODO: set to None, hardcoded only for testing
        self._docked_pose: GeoPose | None = GeoPose.from_degrees(lat=51.982607, lon=7.434085, heading=-28.8)
        self._approach_pose: GeoPose | None = GeoPose.from_degrees(lat=51.982607, lon=7.434085, heading=-28.8) \
            .relative_shift_by(x=self.DOCKING_DISTANCE)

    async def approach(self):
        rosys.notify('Approaching not implemented yet')

    async def dock(self):
        async def wait_for_charging():
            await self.bms.CHARGING_STARTED.emitted()
            self.log.warning('Charging station detected, stopping')

        async def gnss_move():
            assert self.system.gnss is not None
            assert self.system.gnss.last_measurement is not None
            if self.system.gnss.last_measurement.gps_quality != GpsQuality.RTK_FIXED:
                self.log.error('No RTK fix, aborting')
                return
            assert self._docked_pose is not None
            self.log.warning(f'Moving to docked pose: {self._docked_pose}')
            local_docked_pose = self._docked_pose.to_local()
            spline = Spline.from_poses(self.system.robot_locator.pose, local_docked_pose, backward=True)
            with self.system.driver.parameters.set(can_drive_backwards=True, linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_spline(spline, flip_hook=True)

        async def wrapper():
            await rosys.automation.parallelize(
                gnss_move(),
                wait_for_charging(),
                return_when_first_completed=True,
            )
            await self.system.field_friend.wheels.stop()

        rosys.notify('Docking to charging station')
        self.system.automator.start(wrapper())

    async def undock(self):
        async def move():
            assert self._approach_pose is not None
            undock_pose = self._approach_pose.to_local()
            with self.system.driver.parameters.set(linear_speed_limit=self.docking_speed):
                await self.system.driver.drive_to(undock_pose.point)
        if self._approach_pose is None:
            rosys.notify('Record the docked position first', 'negative')
            return
        rosys.notify('Detaching from charging station')
        self.system.automator.start(move())

    def _set_docked_position(self):
        self._docked_pose = GeoPose.from_pose(self.system.robot_locator.pose)
        self._approach_pose = self._docked_pose.relative_shift_by(x=self.DOCKING_DISTANCE)
        self.log.warning('Setting docked pose to %s', self._docked_pose)

    def developer_ui(self):
        with ui.column():
            ui.label('Charging Station').classes('text-center text-bold')
            ui.number(label='Docking distance', min=0, step=0.01, format='%.3f', suffix='m', value=self.docking_distance) \
                .classes('w-4/5').bind_value_to(self, 'docking_distance')
            ui.number(label='Docking speed', min=0, step=0.01, format='%.2f', suffix='m/s', value=self.docking_speed) \
                .classes('w-4/5').bind_value_to(self, 'docking_speed')

            ui.button('Dock', on_click=self.dock)
            ui.button('Undock', on_click=self.undock)
            ui.button('Set Docked Position', on_click=self._set_docked_position)
            ui.label('Charging').bind_text_from(self.system.field_friend.bms.state, 'is_charging',
                                                lambda is_charging: 'Charging' if is_charging else 'Not charging')
