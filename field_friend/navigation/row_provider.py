from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Optional

import rosys
from rosys import persistence
from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Row:
    name: str
    point1: Point
    point2: Point
    reverse: bool = False
    reference_lat: Optional[float] = None
    reference_lon: Optional[float] = None

    @property
    def start(self) -> Point:
        return self.point1 if not self.reverse else self.point2

    @property
    def end(self) -> Point:
        return self.point2 if not self.reverse else self.point1


class RowProvider:

    def __init__(self) -> None:
        self.rows: list[Row] = []

        self.ROWS_CHANGED = rosys.event.Event()
        """The list of rows has changed"""

        self.needs_backup: bool = False
        persistence.register(self)

    def backup(self) -> dict:
        return {'rows': persistence.to_dict(self.rows)}

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_list(self.rows, Row, data.get('rows', []))

    def invalidate(self) -> None:
        self.needs_backup = True
        self.ROWS_CHANGED.emit()

    def add_row(self, row: Row) -> None:
        self.rows.append(row)
        self.invalidate()

    def remove_row(self, row: Row) -> None:
        self.rows.remove(row)
        self.invalidate()

    def clear_rows(self) -> None:
        self.rows.clear()
        self.invalidate()
