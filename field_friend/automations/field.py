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

from .. import localization


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
class RowSupportPoint(GeoPoint):
    row_index: int

    @classmethod
    def from_geopoint(cls, geopoint: GeoPoint, row_index: int) -> Self:
        return cls(lat=geopoint.lat, long=geopoint.long, row_index=row_index)


class Field:
    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str,
                 first_row_start: GeoPoint,
                 first_row_end: GeoPoint,
                 row_spacing: float = 0.5,
                 row_number: int = 10,
                 outline_buffer_width: float = 2,
                 row_support_points: list[RowSupportPoint] | None = None) -> None:
        self.id: str = id
        self.name: str = name
        self.first_row_start: GeoPoint = first_row_start
        self.first_row_end: GeoPoint = first_row_end
        self.row_spacing: float = row_spacing
        self.row_number: int = row_number
        self.outline_buffer_width: float = outline_buffer_width
        self.row_support_points: list[RowSupportPoint] = row_support_points or []
        self.row_support_points.sort(key=lambda sp: sp.row_index)
        self.visualized: bool = False
        self.rows: list[Row] = []
        self.outline: list[GeoPoint] = []
        self.refresh()

    @property
    def outline_cartesian(self) -> list[rosys.geometry.Point]:
        return [p.cartesian() for p in self.outline]

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
        self.rows = self._generate_rows()
        self.outline = self._generate_outline()

    def _generate_rows(self) -> list[Row]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        ab_line_cartesian = LineString([self.first_row_start.cartesian().tuple, self.first_row_end.cartesian().tuple])
        rows: list[Row] = []

        last_support_point = None
        last_support_point_offset = 0

        for i in range(int(self.row_number)):
            support_point = next((sp for sp in self.row_support_points if sp.row_index == i), None)
            if support_point:
                support_point_cartesian = support_point.cartesian()
                offset = ab_line_cartesian.distance(shapely.geometry.Point(
                    [support_point_cartesian.x, support_point_cartesian.y]))
                last_support_point = support_point
                last_support_point_offset = offset
            else:
                if last_support_point:
                    offset = last_support_point_offset + (i - last_support_point.row_index) * self.row_spacing
                else:
                    offset = i * self.row_spacing
            offset_row_coordinated = offset_curve(ab_line_cartesian, -offset).coords
            row_points: list[GeoPoint] = [localization.reference.shifted(
                Point(x=p[0], y=p[1])) for p in offset_row_coordinated]
            row = Row(id=f'field_{self.id}_row_{str(i + 1)}', name=f'row_{i + 1}', points=row_points)
            rows.append(row)
        return rows

    def _generate_outline(self) -> list[GeoPoint]:
        assert len(self.rows) > 0
        outline_unbuffered: list[Point] = []
        for p in self.rows[-1].points:
            outline_unbuffered.append(p.cartesian())
        outline_unbuffered.append(self.first_row_end.cartesian())
        outline_unbuffered.append(self.first_row_start.cartesian())
        outline_polygon = Polygon([p.tuple for p in outline_unbuffered])
        bufferd_polygon = outline_polygon.buffer(
            self.outline_buffer_width, join_style='mitre', mitre_limit=math.inf)
        bufferd_polygon_coords = bufferd_polygon.exterior.coords
        outline: list[GeoPoint] = [localization.reference.shifted(
            Point(x=p[0], y=p[1])) for p in bufferd_polygon_coords]
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
            'row_support_points': [rosys.persistence.to_dict(sp) for sp in self.row_support_points],
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
        data['row_support_points'] = [rosys.persistence.from_dict(
            RowSupportPoint, sp) for sp in data['row_support_points']] if 'row_support_points' in data else []
        field_data = cls(**cls.args_from_dict(data))
        return field_data
