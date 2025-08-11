from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any

import rosys
from nicegui import ui
from nicegui.elements.scene_objects import Sphere
from rosys.geometry import GeoPose, Spline
from rosys.hardware import BmsSimulation
from rosys.hardware.gnss import GpsQuality

from .navigation import DriveSegment

if TYPE_CHECKING:
    from field_friend.system import System


class ChargingStation:
    DOCKING_DISTANCE = 3.0
    DOCKING_SPEED = 0.1

    MINIMUM_CHARGE_PERCENTAGE = 0.2
    MAXIMUM_CHARGE_PERCENTAGE = 0.90

    def __init__(self, system: System, *, docked_pose: GeoPose | None = None, docking_distance: float = DOCKING_DISTANCE):
        self.log = logging.getLogger(__name__)
        self.system = system
        self.bms = system.field_friend.bms

        self.docking_distance = docking_distance
        self._docked_pose: GeoPose | None = docked_pose or GeoPose.from_degrees(
            lat=51.983158, lon=7.434479, heading=40.9)
        self._approach_pose: GeoPose | None = self._docked_pose.relative_shift_by(x=self.docking_distance)

    @property
    def is_fully_charged(self) -> bool:
        assert self.bms.state is not None
        assert self.bms.state.percentage is not None
        return self.bms.state.percentage >= self.MAXIMUM_CHARGE_PERCENTAGE

    @property
    def is_charging(self) -> bool:
        assert self.bms.state is not None
        return self.bms.state.is_charging or False

    @property
    def docked_pose(self) -> GeoPose | None:
        return self._docked_pose

    @property
    def approach_pose(self) -> GeoPose | None:
        return self._approach_pose

    async def approach(self, *, segment: DriveSegment | None = None):
        if segment is None:
            if self._approach_pose is None:
                rosys.notify('Record the docked position first', 'negative')
                return
            assert self._approach_pose is not None
            segment = DriveSegment.from_poses(self.system.robot_locator.pose,
                                              self._approach_pose.to_local())

        with self.system.driver.parameters.set(linear_speed_limit=0.3, can_drive_backwards=segment.backward):
            await self.system.driver.drive_spline(segment.spline,
                                                  flip_hook=segment.backward,
                                                  throttle_at_end=segment.stop_at_end,
                                                  stop_at_end=segment.stop_at_end)
        rosys.notify('Detaching from charging station')

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
            with self.system.driver.parameters.set(can_drive_backwards=True, linear_speed_limit=self.DOCKING_SPEED):
                await self.system.driver.drive_spline(spline, flip_hook=True)
            if isinstance(self.bms, BmsSimulation):
                self.bms.voltage_per_second = 0.03

        rosys.notify('Docking to charging station')
        await rosys.automation.parallelize(
            gnss_move(),
            wait_for_charging(),
            return_when_first_completed=True,
        )
        await self.system.field_friend.wheels.stop()

    async def undock(self):
        if self._approach_pose is None:
            rosys.notify('Record the docked position first', 'negative')
            return
        rosys.notify('Detaching from charging station')
        assert self._approach_pose is not None
        undock_pose = self._approach_pose.to_local()
        if isinstance(self.bms, BmsSimulation):
            self.bms.voltage_per_second = -0.01
        with self.system.driver.parameters.set(linear_speed_limit=self.DOCKING_SPEED):
            await self.system.driver.drive_to(undock_pose.point)

    def _set_docked_position(self, *, docked_pose: GeoPose | None = None):
        self._docked_pose = docked_pose or GeoPose.from_pose(self.system.robot_locator.pose)
        self._approach_pose = self._docked_pose.relative_shift_by(x=self.DOCKING_DISTANCE)
        self.log.warning('Setting docked pose to %s', self._docked_pose)

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'docked_pose': self._docked_pose.degree_tuple if self._docked_pose is not None else None,
            'docking_distance': self.docking_distance,
        }

    def developer_ui(self):
        with ui.column():
            ui.label('Charging Station').classes('text-center text-bold')
            ui.number(label='Docking distance', min=0, step=0.01, format='%.3f', suffix='m', value=self.docking_distance) \
                .classes('w-4/5').bind_value_to(self, 'docking_distance')
            ui.button('Approach', on_click=self.approach)
            ui.button('Dock', on_click=self.dock)
            ui.button('Undock', on_click=self.undock)
            ui.button('Set Docked Position', on_click=self._set_docked_position)

    def scene_object(self) -> None:
        if self._docked_pose is None or self._approach_pose is None:
            return
        local_docked_pose = self._docked_pose.to_local()
        Sphere(radius=0.1).move(x=local_docked_pose.x, y=local_docked_pose.y, z=0.1) \
            .material('#ff0000').with_name('docked_pose')
        local_approach_pose = self._approach_pose.to_local()
        Sphere(radius=0.1).move(x=local_approach_pose.x, y=local_approach_pose.y, z=0.1) \
            .material('#008000').with_name('approach_pose')
