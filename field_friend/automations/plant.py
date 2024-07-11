import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Optional
from uuid import uuid4

from rosys.geometry import Point
from rosys.vision import Image


@dataclass(slots=True, kw_only=True)
class Plant:
    id: str = field(default_factory=lambda: str(uuid4()))
    type: str
    positions: deque[Point] = field(default_factory=lambda: deque(maxlen=20))
    detection_time: float
    confidences: deque[float] = field(default_factory=lambda: deque(maxlen=20))
    detection_image: Optional[Image] = None

    @property
    def position(self) -> Point:
        """Calculate the middle position of all points"""
        total_x = sum(point.x for point in self.positions)
        total_y = sum(point.y for point in self.positions)

        middle_x = total_x / len(self.positions)
        middle_y = total_y / len(self.positions)

        return Point(x=middle_x, y=middle_y)

    @property
    def confidence(self) -> float:
        # TODO: maybe use weighted confidence
        # sum_confidence = sum(confidence**1.5 for confidence in self.confidences)
        sum_confidence = sum(self.confidences)
        return sum_confidence
