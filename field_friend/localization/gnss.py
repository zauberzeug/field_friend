from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rosys

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
    heading: Optional[float] = None
    speed_kmh: float = 0.0


class Gnss(ABC):

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

        self.current: Optional[GNSSRecord] = None
        self.device: str | None = None
        self.antenna_offset = antenna_offset
        self.is_paused = False
        self.observed_poses: list[rosys.geometry.Pose] = []
        self.last_pose_update = rosys.time()
        self.min_seconds_between_updates = 10.0
        self.ensure_gnss = False

        self.needs_backup = False
        rosys.on_repeat(self.check_gnss, 0.01)
        rosys.on_repeat(self.try_connection, 3.0)

    @abstractmethod
    async def try_connection(self) -> None:
        pass

    async def check_gnss(self) -> None:
        if self.is_paused:
            return
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
            if previous.gps_qual == 4 and self.current.gps_qual != 4:
                self.log.warning('GNSS RTK fix lost')
                self.ROBOT_GNSS_POSITION_CHANGED.emit(self.current.location)
                self.RTK_FIX_LOST.emit()
                return
        if self.current is None:
            return
        try:
            # TODO also do antenna_offset correction for this event
            self.ROBOT_GNSS_POSITION_CHANGED.emit(self.current.location)
            if self.current.gps_qual == 4:  # 4 = RTK fixed (cm accuracy), 5 = RTK float (dm accuracy)
                self._on_rtk_fix()
        except Exception:
            self.log.exception('gnss record could not be applied')
            self.current = None

    @abstractmethod
    async def _create_new_record(self) -> Optional[GNSSRecord]:
        pass

    def _on_rtk_fix(self) -> None:
        assert self.current is not None
        if localization.reference.lat == 0 and localization.reference.long == 0:
            self.log.info(f'GNSS reference set to {self.current.location}')
            localization.reference = deepcopy(self.current.location)
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
        assert not self.is_paused
        if self.ensure_gnss:
            while len(self.observed_poses) < 10:
                if rosys.time() - self.last_pose_update < self.min_seconds_between_updates:
                    return
                await rosys.sleep(0.1)
        if not self.observed_poses:
            return
        self.log.info(f'updating robot pose from {len(self.observed_poses)} GNSS records')
        x = np.mean([pose.point.x for pose in self.observed_poses])
        y = np.mean([pose.point.y for pose in self.observed_poses])
        yaw = np.mean([pose.yaw for pose in self.observed_poses])
        pose = rosys.geometry.Pose(x=float(x), y=float(y), yaw=float(yaw), time=rosys.time())
        self.ROBOT_POSE_LOCATED.emit(pose)
        self.last_pose_update = rosys.time()
        self.observed_poses.clear()
