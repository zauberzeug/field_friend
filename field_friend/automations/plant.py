import uuid
from dataclasses import dataclass

from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Plant:
    id: str = ...
    type: str
    position: Point
    detection_time: float

    def __post_init__(self) -> None:
        """Generate a unique ID if not already loaded from persistence"""
        if self.id == ...:
            self.id = str(uuid.uuid4())
