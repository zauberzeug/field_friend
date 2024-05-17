from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional, Protocol

import numpy as np
import rosys
import serial

from .geo_point import GeoPoint
from .point_transformation import get_new_position


@dataclass
class GNSSRecord:
    timestamp: float = 0.0
    latitude: float = 0.0
    longitude: float = 0.0
    mode: str = ''
    gps_qual: int = 0
    altitude: float = 0.0
    separation: float = 0.0
    heading: Optional[float] = None
    speed_kmh: float = 0.0

    @property
    def has_location(self):
        return self.latitude is not None and self.longitude is not None


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

        self.current = GNSSRecord()
        self.device: str | None = None
        self.ser: serial.Serial | None = None
        self.reference: Optional[GeoPoint] = None
        self.antenna_offset = antenna_offset

        self.needs_backup = False
        rosys.on_repeat(self.update, 0.01)
        rosys.on_repeat(self.try_connection, 3.0)

    @abstractmethod
    async def update(self) -> None:
        pass

    @abstractmethod
    async def try_connection(self) -> None:
        pass

    def set_reference(self, point: GeoPoint) -> None:
        self.reference = point

    def clear_reference(self) -> None:
        self.reference = None

    def get_reference(self) -> Optional[GeoPoint]:
        return self.reference

    def distance(self, point: GeoPoint) -> Optional[float]:
        """Compute the distance between the reference point and the given point in meters"""
        if self.reference is None:
            return None
        return point.distance(point)

    def _update_record(self, new: GNSSRecord) -> None:
        previous = deepcopy(self.current)
        self.current = deepcopy(new)
        if new.gps_qual == 0:
            if previous.gps_qual != 0:
                self.log.warning('GNSS lost')
                self.GNSS_CONNECTION_LOST.emit()
            return
        if previous.gps_qual == 4 and new.gps_qual != 4:
            self.log.warning('GNSS RTK fix lost')
            self.RTK_FIX_LOST.emit()
            return
        if not self.current.has_location:
            return
        geo_point = GeoPoint(lat=self.current.latitude, long=self.current.longitude)
        self.ROBOT_GNSS_POSITION_CHANGED.emit(geo_point)  # TODO also do antenna_offset correction for this event
        if self.current.gps_qual != 4:  # 4 = RTK fixed (cm accuracy), 5 = RTK float (dm accuracy)
            return
        if self.reference is None:
            self.log.info(f'GNSS reference set to {self.current.latitude}, {self.current.longitude}')
            self.set_reference(GeoPoint(lat=self.current.latitude, long=self.current.longitude))

        if self.current.heading is not None:
            yaw = np.deg2rad(-self.current.heading)
        else:
            # TODO: Better INS implementation if no heading provided by GNSS
            yaw = self.odometer.get_pose(time=self.current.timestamp).yaw
        # correct the gnss coordinat by antenna offset
        corrected_coordinates = get_new_position([self.current.latitude, self.current.longitude],
                                                 self.antenna_offset, yaw+np.pi/2)
        self.current.latitude = deepcopy(corrected_coordinates[0])
        self.current.longitude = deepcopy(corrected_coordinates[1])
        cartesian_coordinates = geo_point.cartesian(self.reference)
        pose = rosys.geometry.Pose(
            x=cartesian_coordinates.x,
            y=cartesian_coordinates.y,
            yaw=yaw,
            time=self.current.timestamp)
        distance = self.odometer.prediction.distance(pose)
        if distance > 1:
            self.log.warning(f'GNSS distance to prediction too high: {distance:.2f}m!!')
        self.ROBOT_POSE_LOCATED.emit(pose)


class PoseProvider(Protocol):

    @property
    def pose(self) -> rosys.geometry.Pose:
        ...
