import uuid
from collections import deque
from dataclasses import dataclass
from typing import Optional

from rosys.geometry import Point
from rosys.vision import Image


@dataclass(slots=True, kw_only=True)
class Plant:
    id: str = ...
    type: str
    positions: deque[Point]
    detection_time: float
    confidence: float = 0.0
    detection_image: Optional[Image] = None

    def __init__(self, type_: str, position: Point, detection_time: float, id_: str = ..., confidence: float = 0.0, max_positions: int = 20, detection_image: Optional[Image] = None) -> None:
        self.id = id_
        self.type = type_
        self.positions = deque([position], maxlen=max_positions)
        self.detection_time = detection_time
        self.confidence = confidence
        self.detection_image = detection_image

    def __post_init__(self) -> None:
        """Generate a unique ID if not already loaded from persistence"""
        if self.id == ...:
            self.id = str(uuid.uuid4())

    @property
    def position(self) -> Point:
        """Calculate the middle position of all points"""
        total_x = sum(point.x for point in self.positions)
        total_y = sum(point.y for point in self.positions)

        middle_x = total_x / len(self.positions)
        middle_y = total_y / len(self.positions)

        return Point(x=middle_x, y=middle_y)
