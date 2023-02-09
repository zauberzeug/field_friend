import uuid
from dataclasses import dataclass
from typing import Optional

from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Plant:
    id: Optional[str] = None
    type: str
    position: Point
    mac: str
    detection_time: float
    outline: Optional[list[tuple[float, float]]] = None

    def __post_init__(self) -> None:
        self.id = self.id or str(uuid.uuid4())

    def __str__(self) -> str:
        return f'plant with id: {self.id[:5]}, pos: {self.position}, mac: {self.mac}'
