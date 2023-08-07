from dataclasses import dataclass, field
from typing import Any, Optional

import rosys
from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class FieldObstacle:
    name: str
    points: list[Point] = field(default_factory=list)


@dataclass(slots=True, kw_only=True)
class Field:
    name: str
    outline: list[Point] = field(default_factory=list)
    reference_lat: Optional[float] = None
    reference_lon: Optional[float] = None
    visualized: bool = False
    obstacles: list[FieldObstacle] = field(default_factory=list)


class FieldProvider:
    def __init__(self) -> None:
        self.fields: list[Field] = []

        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

        self.needs_backup: bool = False
        rosys.persistence.register(self)

    def backup(self) -> dict:
        return {'fields': rosys.persistence.to_dict(self.fields)}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.persistence.replace_list(self.fields, Field, data.get('fields', []))

    def invalidate(self) -> None:
        self.needs_backup = True
        self.FIELDS_CHANGED.emit()

    def add_field(self, field: Field) -> None:
        self.fields.append(field)
        self.invalidate()

    def remove_field(self, field: Field) -> None:
        self.fields.remove(field)
        self.invalidate()

    def clear_fields(self) -> None:
        self.fields.clear()
        self.invalidate()

    def add_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.append(obstacle)
        self.invalidate()

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.remove(obstacle)
        self.invalidate()
