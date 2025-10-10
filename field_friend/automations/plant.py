from collections import deque
from dataclasses import dataclass, field
from typing import ClassVar
from uuid import uuid4

from rosys.geometry import Point3d
from rosys.vision import Image


@dataclass(slots=True, kw_only=True)
class Plant:
    DEQUE_MAXLEN: ClassVar[int] = 20
    id: str = field(default_factory=lambda: str(uuid4()))
    type: str
    positions: deque[Point3d] = field(default_factory=lambda: deque(maxlen=Plant.DEQUE_MAXLEN))
    detection_time: float
    confidences: deque[float] = field(default_factory=lambda: deque(maxlen=Plant.DEQUE_MAXLEN))
    detection_image: Image | None = None

    @property
    def position(self) -> Point3d:
        """Calculate the middle position of all points"""
        total_x = sum(point3d.x for point3d in self.positions)
        total_y = sum(point3d.y for point3d in self.positions)
        total_z = sum(point3d.z for point3d in self.positions)

        middle_x = total_x / len(self.positions)
        middle_y = total_y / len(self.positions)
        middle_z = total_z / len(self.positions)

        return Point3d(x=middle_x, y=middle_y, z=middle_z)

    @property
    def confidence(self) -> float:
        # TODO: maybe use weighted confidence
        # sum_confidence = sum(confidence**1.5 for confidence in self.confidences)
        sum_confidence = sum(self.confidences)
        return sum_confidence
