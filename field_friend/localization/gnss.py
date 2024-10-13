from __future__ import annotations

import logging
import os
from abc import ABC, abstractmethod
from collections import deque
from copy import deepcopy
from dataclasses import dataclass
from typing import Any

import numpy as np
import rosys
import rosys.helpers
from nicegui import ui
from rosys.persistence.registry import backup

from .. import localization
from .geo_point import GeoPoint
from .point_transformation import get_new_position


@dataclass
class GNSSRecord:
    timestamp: float
    location: GeoPoint
    mode: str = ''
    gps_qual: int = 0
    altitude: float = 0.0
    separation: float = 0.0
    heading: float | None = None
    speed_kmh: float = 0.0


class Gnss(rosys.persistence.PersistentModule, ABC):
    NEEDED_POSES: int = 10
    MAX_POSES: int = 30
    MAX_DISTANCE_TO_REFERENCE: float = 5000.0

    def __init__(self, odometer: rosys.driving.Odometer, antenna_offset: float) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.gnss')
        self.odometer = odometer

        self.ROBOT_POSE_LOCATED = rosys.event.Event()
        """the robot has been located (argument: Pose) with RTK-fixed"""

        self.ROBOT_GNSS_POSITION_CHANGED = rosys.event.Event()
        """the robot has been located (argument: GeoPoint), may only be a rough estimate if no RTK fix is available"""

        self.RTK_FIX_LOST = rosys.event.Event()
        """the robot lost RTK fix"""

        self.GNSS_CONNECTION_LOST = rosys.event.Event()
        """the GNSS connection was lost"""

        self.current: GNSSRecord | None = None
        self.device: str | None = None
        self.antenna_offset = antenna_offset
        self.observed_poses: deque[rosys.geometry.Pose] = deque(maxlen=self.MAX_POSES)
        self.needed_poses: int = self.NEEDED_POSES
        self.max_distance_to_reference: float = self.MAX_DISTANCE_TO_REFERENCE
        self.reference_alert_dialog: ui.dialog
        self._last_odometer_pose = self.odometer.prediction
        self._last_gnss_pose = self.odometer.prediction

        self.needs_backup = False
        rosys.on_repeat(self.check_gnss, 0.01)
        rosys.on_repeat(self.try_connection, 3.0)
        rosys.on_repeat(self.update_robot_pose, 0.5)

    @abstractmethod
    async def try_connection(self) -> None:
        pass

    async def check_gnss(self) -> None:
        previous = deepcopy(self.current)
        try:
            self.current = await self._create_new_record()
        except Exception:
            self.log.exception('creation of gnss record failed')
        if previous is not None:
            if self.current is None:
                self.log.warning('new GNSS record is None')
                self.GNSS_CONNECTION_LOST.emit()
                return
            if ("R" in previous.mode or previous.mode == "SSSS") and ("R" not in self.current.mode and self.current.mode != "SSSS"):
                self.log.warning('GNSS RTK fix lost')
                self.ROBOT_GNSS_POSITION_CHANGED.emit(self.current.location)
                self.RTK_FIX_LOST.emit()
                return
        if self.current is None:
            return
        try:
            # TODO also do antenna_offset correction for this event
            self.ROBOT_GNSS_POSITION_CHANGED.emit(self.current.location)
            if "R" in self.current.mode or self.current.mode == "SSSS":
                await self._on_rtk_fix()
        except Exception:
            self.log.exception('gnss record could not be applied')
            self.current = None

    @abstractmethod
    async def _create_new_record(self) -> GNSSRecord | None:
        pass

    async def _on_rtk_fix(self) -> None:
        distance = self._last_odometer_pose.distance(self.odometer.prediction)
        yaw_difference = abs(rosys.helpers.angle(self._last_odometer_pose.yaw, self.odometer.prediction.yaw))
        if abs(distance) > 0.0001 or yaw_difference > np.deg2rad(0.01):  # if robot does not stand still
            self.observed_poses.clear()
            self._last_odometer_pose = self.odometer.prediction
            return
        assert self.current is not None
        if localization.reference.lat == 0 and localization.reference.long == 0:
            await self.update_reference()
        if self.current.heading is not None:
            yaw = np.deg2rad(-self.current.heading)
        else:
            # TODO: Better INS implementation if no heading provided by GNSS
            yaw = self.odometer.get_pose(time=self.current.timestamp).yaw
        # correct the gnss coordinate by antenna offset
        self.current.location = get_new_position(self.current.location, self.antenna_offset, yaw+np.pi/2)
        cartesian_coordinates = self.current.location.cartesian()
        pose = rosys.geometry.Pose(
            x=cartesian_coordinates.x,
            y=cartesian_coordinates.y,
            yaw=yaw,
            time=self.current.timestamp)
        self.observed_poses.append(pose)

    async def update_robot_pose(self) -> None:
        if len(self.observed_poses) < self.needed_poses:
            return
        x = np.mean([pose.point.x for pose in self.observed_poses])
        y = np.mean([pose.point.y for pose in self.observed_poses])
        yaw = np.mean([pose.yaw for pose in self.observed_poses])
        self._last_gnss_pose = rosys.geometry.Pose(x=float(x), y=float(
            y), yaw=float(yaw), time=self.observed_poses[-1].time)
        self.ROBOT_POSE_LOCATED.emit(self._last_gnss_pose)
        self.observed_poses.clear()

    def backup(self) -> dict:
        return {
            'needed_poses': self.needed_poses,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.needed_poses = data.get('needed_poses', self.needed_poses)

    async def update_reference(self) -> None:
        if self.current is None:
            self.log.warning('No GNSS position available')
            return
        localization.reference = self.current.location
        await backup(force=True)
        self.log.info('GNSS reference set to %s', self.current.location)
        os.utime('main.py')

    def reference_warning_dialog(self) -> None:
        with ui.dialog() as self.reference_alert_dialog, ui.card():
            ui.label('The reference is to far away from the current position which would lead to issues in the navigation. Do you want to set it now?')
            with ui.row():
                ui.button("Update reference", on_click=self.update_reference).props("outline color=warning") \
                    .tooltip("Set current position as geo reference and restart the system").classes("ml-auto").style("display: block; margin-top:auto; margin-bottom: auto;")
                ui.button('Cancel', on_click=self.reference_alert_dialog.close)

    def check_distance_to_reference(self) -> bool:
        if self.current is not None and self.current.location.distance(localization.reference) > self.max_distance_to_reference:
            self.reference_alert_dialog.open()
            return True
        return False

    @ui.refreshable
    def developer_ui(self) -> None:
        ui.label('GNSS').classes('text-center text-bold')
        ui.label(f'Reference: {localization.reference}')
        ui.label(f'Observed_Poses: {len(self.observed_poses)}')
        ui.label(f'Needed Poses: {self.needed_poses}')
        ui.label(f'odom: {self.odometer.prediction}')
