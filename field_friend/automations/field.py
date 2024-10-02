import math
from dataclasses import dataclass
from typing import Any
from uuid import uuid4

import rosys
import shapely
from rosys.geometry import Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon
from typing_extensions import Self

from field_friend.localization import GeoPoint, GeoPointCollection


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


class Field:
    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str,
                 first_row_start: GeoPoint,
                 first_row_end: GeoPoint,
                 row_spacing: float = 0.5,
                 row_number: int = 10,
                 outline_buffer_width: float = 2) -> None:
        self.id: str = id
        self.name: str = name
        self.first_row_start: GeoPoint = first_row_start
        self.first_row_end: GeoPoint = first_row_end
        self.row_spacing: float = row_spacing
        self.row_number: int = row_number
        self.outline_buffer_width: float = outline_buffer_width
        self.visualized: bool = False
        self.rows: list[Row] = []
        self.outline: list[GeoPoint] = []
        self.refresh()

    @property
    def outline_cartesian(self) -> list[rosys.geometry.Point]:
        cartesian_points = []
        for point in self.outline:
            cartesian_points.append(point.cartesian())
        return cartesian_points

    @property
    def outline_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline]

    @property
    def outline_cartesian_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline_cartesian]

    def area(self) -> float:
        outline_cartesian = self.outline_cartesian
        if not outline_cartesian:
            return 0.0
        polygon = Polygon([(p.x, p.y) for p in outline_cartesian])
        return polygon.area

    def worked_area(self, worked_rows: int) -> float:
        worked_area = 0.0
        if self.area() > 0:
            worked_area = worked_rows * self.area() / self.row_number
        return worked_area

    def refresh(self):
        self.outline = self._generate_outline()
        self.rows = self._generate_rows()

    @property
    def rows(self) -> list[Row]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        ab_line_cartesian = LineString([self.first_row_start.cartesian().tuple, self.first_row_end.cartesian().tuple])
        rows = []
        for i in range(int(self.row_number)):
            offset = i * self.row_spacing
            offset_row_coordinated = offset_curve(ab_line_cartesian, -offset).coords
            row_points: list[GeoPoint] = []
            for point in offset_row_coordinated:
                row_points.append(self.first_row_start.shifted(Point(x=point[0], y=point[1])))
            row = Row(id=str(uuid4()), name=f'{i + 1}', points=row_points)
            rows.append(row)
        return rows

    def _generate_outline(self) -> list[GeoPoint]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        ab_line_cartesian = LineString([self.first_row_start.cartesian().tuple, self.first_row_end.cartesian().tuple])
        last_row_linestring = offset_curve(ab_line_cartesian, - self.row_spacing * self.row_number + self.row_spacing)
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

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'first_row_start':  rosys.persistence.to_dict(self.first_row_start),
            'first_row_end': rosys.persistence.to_dict(self.first_row_end),
            'row_spacing': self.row_spacing,
            'row_number': self.row_number,
            'outline_buffer_width': self.outline_buffer_width,
        }

    def shapely_polygon(self) -> shapely.geometry.Polygon:
        return shapely.geometry.Polygon([p.tuple for p in self.outline])

    @classmethod
    def args_from_dict(cls, data: dict[str, Any]) -> dict:
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        data['first_row_start'] = GeoPoint(lat=data['first_row_start']['lat'], long=data['first_row_start']['long'])
        data['first_row_end'] = GeoPoint(lat=data['first_row_end']['lat'], long=data['first_row_end']['long'])
        field_data = cls(**cls.args_from_dict(data))
        return field_data
