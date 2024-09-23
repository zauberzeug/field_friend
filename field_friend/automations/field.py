import math
from dataclasses import dataclass, field
from uuid import uuid4

import rosys
from rosys.geometry import Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon

from field_friend.localization import GeoPoint, GeoPointCollection, reference


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

    def line_segment(self) -> rosys.geometry.LineSegment:
        return rosys.geometry.LineSegment(point1=self.points[0].cartesian(),
                                          point2=self.points[-1].cartesian())


@dataclass(slots=True, kw_only=True)
class Field(GeoPointCollection):
    visualized: bool = False
    first_row_start: GeoPoint | None = None
    first_row_end: GeoPoint | None = None
    row_spacing: float = 0.5
    row_number: int = 10
    obstacles: list[FieldObstacle] = field(default_factory=list)
    outline_buffer_width: float = 2

    @property
    def outline(self) -> list[GeoPoint]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        ab_line_cartesian = LineString([self.first_row_start.cartesian().tuple, self.first_row_end.cartesian().tuple])
        last_row_linestring = offset_curve(ab_line_cartesian, - self.row_spacing * self.row_number)
        end_row_points: list[Point] = []
        for point in last_row_linestring.coords:
            end_row_points.append(Point(x=point[0], y=point[1]))
        outline_unbuffered: list[Point] = []
        for i, point in enumerate(end_row_points):
            outline_unbuffered.append(point)
        outline_unbuffered.append(self.first_row_end.cartesian())
        outline_unbuffered.append(self.first_row_start.cartesian())
        outline_polygon = Polygon([p.tuple for p in outline_unbuffered])
        bufferd_polygon = outline_polygon.buffer(
            self.outline_buffer_width, join_style='mitre', mitre_limit=math.inf)
        bufferd_polygon_coords = bufferd_polygon.exterior.coords
        outline: list[GeoPoint] = []
        for p in bufferd_polygon_coords:
            outline.append(self.first_row_start.shifted(Point(x=p[0], y=p[1])))
        return outline

    @ property
    def outline_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline]

    def area(self) -> float:
        outline = self.outline
        if not outline:
            return 0.0
        polygon = Polygon([(p.lat, p.long) for p in outline])
        return polygon.area

    def worked_area(self, worked_rows: int) -> float:
        worked_area = 0.0
        if self.area() > 0:
            worked_area = worked_rows * self.area() / len(self.rows)
        return worked_area

    @ property
    def rows(self) -> list[Row]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        first_row_start_cartesian = Point(x=0, y=0)  # first_row_start as reference for calculation
        first_row_end_cartesian = self.first_row_end.cartesian()
        ab_line_cartesian = LineString([first_row_start_cartesian.tuple, first_row_end_cartesian.tuple])
        rows = []
        for i in range(int(self.row_number)+1):
            offset = i * self.row_spacing
            offset_row_coordinated = offset_curve(ab_line_cartesian, -offset).coords
            row_points: list[GeoPoint] = []
            for point in offset_row_coordinated:
                row_points.append(self.first_row_start.shifted(Point(x=point[0], y=point[1])))
            row = Row(id=str(uuid4()), name=f'{i + 1}', points=row_points)
            rows.append(row)
        return rows
