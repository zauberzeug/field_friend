import uuid
from dataclasses import dataclass, field

from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Plant:
    id: str = field(init=False)
    type: str
    position: Point
    detection_time: float

    def __post_init__(self) -> None:
        self.id = str(uuid.uuid4())
