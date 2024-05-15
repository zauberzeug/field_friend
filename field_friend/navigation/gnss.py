from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Protocol

import rosys
import serial

from .geo_point import GeoPoint


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

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
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

    def get_reference(self) -> tuple[Optional[float], Optional[float]]:
        return self.reference_lat, self.reference_lon

    def distance(self, point: GeoPoint) -> Optional[float]:
        """Compute the distance between the reference point and the given point in meters"""
        if self.reference_lat is None or self.reference_lon is None:
            return None
        return GeoPoint(lat=self.reference_lat, long=self.reference_lon).distance(point)


class PoseProvider(Protocol):

    @property
    def pose(self) -> rosys.geometry.Pose:
        ...
