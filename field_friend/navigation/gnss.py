from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Protocol

import rosys
import serial
from geographiclib.geodesic import Geodesic


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


class Gnss(ABC):

    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.gnss')

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
        self.reference_lat: Optional[float] = None
        self.reference_lon: Optional[float] = None

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
    def set_reference(self, lat: float, lon: float) -> None:
        pass

    def clear_reference(self) -> None:
        self.reference_lat = None
        self.reference_lon = None

    def get_reference(self) -> tuple[Optional[float], Optional[float]]:
        return self.reference_lat, self.reference_lon

    def calculate_distance(self, lat: float, lon: float) -> Optional[float]:
        if self.reference_lat is None or self.reference_lon is None:
            return None
        geodetic_measurement = Geodesic.WGS84.Inverse(self.reference_lat, self.reference_lon, lat, lon)
        return geodetic_measurement['s12']


class PoseProvider(Protocol):

    @property
    def pose(self) -> rosys.geometry.Pose:
        ...
