from dataclasses import dataclass, field
from typing import Optional

import rosys
from shapely.geometry import Polygon

from field_friend.navigation import GeoPoint, GeoPointCollection

from .plant import Plant


@dataclass(slots=True, kw_only=True)
class FieldObstacle(GeoPointCollection):
    pass


@dataclass(slots=True, kw_only=True)
class Row(GeoPointCollection):
    reverse: bool = False

    def reversed(self):
        return Row(
            id=self.id,
            name=self.name,
            points=list(reversed(self.points)),
        )

    def line_segment(self, reference: GeoPoint) -> rosys.geometry.LineSegment:
        return rosys.geometry.LineSegment(point1=self.points[0].cartesian(reference),
                                          point2=self.points[-1].cartesian(reference))


@dataclass(slots=True, kw_only=True)
class Field(GeoPointCollection):
    reference: Optional[GeoPoint] = None
    visualized: bool = False
    obstacles: list[FieldObstacle] = field(default_factory=list)
    rows: list[Row] = field(default_factory=list)

    @property
    def outline(self) -> list[rosys.geometry.Point]:
        assert self.reference is not None
        return self.cartesian(self.reference)

    @property
    def outline_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline]

    def area(self) -> float:
        outline = self.outline
        if not outline:
            return 0.0
        polygon = Polygon([(p.x, p.y) for p in outline])
        return polygon.area

    def worked_area(self, worked_rows: int) -> float:
        worked_area = 0.0
        if self.area() > 0:
            worked_area = worked_rows * self.area() / len(self.rows)
        return worked_area
