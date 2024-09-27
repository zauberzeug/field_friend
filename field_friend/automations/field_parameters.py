import math
from dataclasses import dataclass
from typing import Any
from uuid import uuid4

from rosys.geometry import Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon
from typing_extensions import Self

from field_friend.localization import GeoPoint

from . import Field, Row


class FieldParameters():

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

    @property
    def field(self) -> 'Field':
        return Field(id=self.id, name=self.name, points=self._generate_outline(), obstacles=[], rows=self._generate_rows())

    def _generate_outline(self) -> list[GeoPoint]:
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

    def _generate_rows(self) -> list[Row]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        first_row_start_cartesian = Point(x=0, y=0)  # first_row_start as reference for calculation
        first_row_end_cartesian = self.first_row_end.cartesian()
        ab_line_cartesian = LineString([first_row_start_cartesian.tuple, first_row_end_cartesian.tuple])
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

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'first_row_start':  self.first_row_start,
            'first_row_end': self.first_row_end,
            'row_spacing': self.row_spacing,
            'row_number': self.row_number,
            'outline_buffer_width': self.outline_buffer_width,
        }

    @classmethod
    def args_from_dict(cls, data: dict[str, Any]) -> dict:
        return data

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**cls.args_from_dict(data))
