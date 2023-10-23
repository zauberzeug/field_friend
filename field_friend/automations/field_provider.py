from dataclasses import dataclass, field
from typing import Any, Optional

import rosys
from rosys.geometry import Point

from .plant import Plant


@dataclass(slots=True, kw_only=True)
class FieldObstacle:
    id: str
    points: list[Point] = field(default_factory=list)


@dataclass(slots=True, kw_only=True)
class Row:
    id: str
    points: list[Point] = field(default_factory=list)
    reverse: bool = False
    crops: list[Plant] = field(default_factory=list)

    def reversed(self):
        return Row(
            id=self.id,
            points=list(reversed(self.points)),
        )


@dataclass(slots=True, kw_only=True)
class Field:
    id: str
    outline: list[Point] = field(default_factory=list)
    reference_lat: Optional[float] = None
    reference_lon: Optional[float] = None
    visualized: bool = False
    obstacles: list[FieldObstacle] = field(default_factory=list)
    rows: list[Row] = field(default_factory=list)


class FieldProvider(rosys.persistence.PersistentModule):
    def __init__(self) -> None:
        self.fields: list[Field] = []

        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

        self.needs_backup: bool = False

    def backup(self) -> dict:
        return {'fields': rosys.persistence.to_dict(self.fields)}

    def restore(self, data: dict[str, Any]) -> None:
        rosys.persistence.replace_list(self.fields, Field, data.get('fields', []))

    def invalidate(self) -> None:
        self.request_backup()
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

    def add_row(self, field: Field, row: Row) -> None:
        field.rows.append(row)
        self.invalidate()

    def remove_row(self, field: Field, row: Row) -> None:
        field.rows.remove(row)
        self.invalidate()
