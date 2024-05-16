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
    heading: float = 0.0
    speed_kmh: float = 0.0

    @property
    def has_location(self):
        return self.latitude is not None and self.longitude is not None

    @property
    def has_heading(self):
        return self.heading is not None


class Gnss(ABC):

    def __init__(self, odometer: rosys.driving.Odometer, antenna_offset: float) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.gnss')
        self.odometer = odometer

        self.ROBOT_POSE_LOCATED = rosys.event.Event()
        """the robot has been located (argument: pose) with RTK-fixed"""

        self.ROBOT_POSITION_LOCATED = rosys.event.Event()
        """the robot has been located"""

        self.RTK_FIX_LOST = rosys.event.Event()
        """the robot lost RTK fix"""

        self.GNSS_CONNECTION_LOST = rosys.event.Event()
        """the GNSS connection was lost"""

        self.record = GNSSRecord()
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

    @abstractmethod
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

    def _update_record(self, record: GNSSRecord) -> None:
        if self.record.gps_qual > 0 and record.gps_qual == 0:
            self.log.warning('GNSS lost')
            self.GNSS_CONNECTION_LOST.emit()
        if self.record.gps_qual == 4 and record.gps_qual != 4:
            self.log.warning('GNSS RTK fix lost')
            self.RTK_FIX_LOST.emit()
        self.record = deepcopy(record)
        if self.record.has_location:
            if record.gps_qual == 4:  # 4 = RTK fixed, 5 = RTK float
                if self.reference is None:
                    self.log.info(f'GNSS reference set to {record.latitude}, {record.longitude}')
                    self.set_reference(GeoPoint(lat=record.latitude, long=record.longitude))
                else:
                    if record.has_heading:
                        yaw = np.deg2rad(-record.heading)
                    else:
                        yaw = self.odometer.get_pose(time=record.timestamp).yaw
                        # TODO: Better INS implementation if no heading provided by GNSS
                    # correct the gnss coordinat by antenna offset
                    corrected_coordinates = get_new_position([record.latitude, record.longitude],
                                                             self.antenna_offset, yaw+np.pi/2)
                    self.record.latitude = deepcopy(corrected_coordinates[0])
                    self.record.longitude = deepcopy(corrected_coordinates[1])
                    assert self.reference is not None
                    cartesian_coordinates = GeoPoint(lat=self.record.latitude, long=self.record.longitude) \
                        .cartesian(self.reference)
                    pose = rosys.geometry.Pose(
                        x=cartesian_coordinates.x,
                        y=cartesian_coordinates.y,
                        yaw=yaw,
                        time=record.timestamp,
                    )
                    distance = self.odometer.prediction.distance(pose)
                    if distance > 1:
                        self.log.warning(f'GNSS distance to prediction to high: {distance:.2f}m!!')
                    self.ROBOT_POSE_LOCATED.emit(pose)
                    self.ROBOT_POSITION_LOCATED.emit()
            elif record.gps_qual == 0:
                return
            else:
                self.ROBOT_POSITION_LOCATED.emit()


class PoseProvider(Protocol):

    @property
    def pose(self) -> rosys.geometry.Pose:
        ...
