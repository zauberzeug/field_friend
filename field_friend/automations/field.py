import math
import uuid
from dataclasses import dataclass
from typing import Any, Self

import rosys
import shapely
from rosys.geometry import GeoPoint, GeoPose, GeoReference, Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon


@dataclass(slots=True, kw_only=True)
class Row:
    id: str
    name: str
    points: list[GeoPoint]
    reverse: bool = False
    crop: str | None = None

    def reversed(self):
        return Row(
            id=self.id,
            name=self.name,
            points=list(reversed(self.points)),
            crop=self.crop
        )

    def line_segment(self) -> rosys.geometry.LineSegment:
        return rosys.geometry.LineSegment(point1=self.points[0].to_local(),
                                          point2=self.points[-1].to_local())


@dataclass(slots=True, kw_only=True)
class RowSupportPoint(GeoPoint):
    row_index: int

    @classmethod
    def from_geopoint(cls, geopoint: GeoPoint, row_index: int) -> Self:
        return cls(lat=geopoint.lat, lon=geopoint.lon, row_index=row_index)


class Field:
    def __init__(self, *,
                 id: str,  # pylint: disable=redefined-builtin
                 name: str,
                 first_row_start: GeoPoint,
                 first_row_end: GeoPoint,
                 row_spacing: float = 0.5,
                 row_count: int = 10,
                 outline_buffer_width: float = 2,
                 row_support_points: list[RowSupportPoint] | None = None,
                 bed_count: int = 1,
                 bed_spacing: float = 0.5,
                 bed_crops: dict[str, str | None] | None = None,
                 docking_distance: float = 2.0,
                 charge_dock_pose: GeoPose | None = None) -> None:
        self.id: str = id
        self.name: str = name
        self.first_row_start: GeoPoint = first_row_start
        self.first_row_end: GeoPoint = first_row_end
        self.row_spacing: float = row_spacing
        self.row_count: int = row_count  # rows per bed
        self.bed_count: int = bed_count
        self.bed_spacing: float = bed_spacing if bed_spacing is not None else row_spacing * 2
        self.outline_buffer_width: float = outline_buffer_width
        self.row_support_points: list[RowSupportPoint] = row_support_points or []
        self.row_support_points.sort(key=lambda sp: sp.row_index)
        self.visualized: bool = False
        self.rows: list[Row] = []
        self.outline: list[GeoPoint] = []
        self.bed_crops: dict[str, str | None] = bed_crops or {str(i): None for i in range(bed_count)}
        self.docking_distance: float = docking_distance
        self._charge_dock_pose: GeoPose | None = None
        self._charge_approach_pose: GeoPose | None = None
        self.charge_dock_pose = charge_dock_pose
        self.refresh()

    @property
    def outline_cartesian(self) -> list[rosys.geometry.Point]:
        return [p.to_local() for p in self.outline]

    @property
    def outline_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline]

    @property
    def outline_cartesian_as_tuples(self) -> list[tuple[float, float]]:
        return [p.tuple for p in self.outline_cartesian]

    @property
    def charge_dock_pose(self) -> GeoPose | None:
        return self._charge_dock_pose

    @charge_dock_pose.setter
    def charge_dock_pose(self, pose: GeoPose | None) -> None:
        self._charge_dock_pose = pose
        self._charge_approach_pose = self._charge_dock_pose.relative_shift_by(x=self.docking_distance) \
            if isinstance(self._charge_dock_pose, GeoPose) else None

    @property
    def charge_approach_pose(self) -> GeoPose | None:
        return self._charge_approach_pose

    @property
    def geo_reference(self) -> GeoReference:
        if self.rows:
            first_row = self.rows[0]
            direction = first_row.points[0].direction(first_row.points[-1])
            return GeoReference(origin=first_row.points[0], direction=direction)
        direction = self.first_row_start.direction(self.first_row_end)
        return GeoReference(origin=self.first_row_start, direction=direction)

    def area(self) -> float:
        outline_cartesian = self.outline_cartesian
        if not outline_cartesian:
            return 0.0
        polygon = Polygon([(p.x, p.y) for p in outline_cartesian])
        return polygon.area

    def worked_area(self, worked_rows: int) -> float:
        worked_area = 0.0
        if self.area() > 0:
            total_rows = self.row_count * self.bed_count
            worked_area = worked_rows * self.area() / total_rows
        return worked_area

    def refresh(self):
        self.rows = self._generate_rows()
        self.outline = self._generate_outline()

    def _generate_rows(self) -> list[Row]:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        ab_line_cartesian = LineString([self.first_row_start.to_local().tuple,
                                        self.first_row_end.to_local().tuple])
        rows: list[Row] = []

        last_support_point = None
        last_support_point_offset = 0

        total_rows = self.row_count * self.bed_count
        for i in range(total_rows):
            bed_index = i // self.row_count
            row_in_bed = i % self.row_count

            support_point = next((sp for sp in self.row_support_points if sp.row_index == i), None)
            if support_point:
                support_point_cartesian = support_point.to_local()
                offset = ab_line_cartesian.distance(shapely.geometry.Point(
                    [support_point_cartesian.x, support_point_cartesian.y]))
                last_support_point = support_point
                last_support_point_offset = offset
            elif last_support_point:
                rows_since_support = i - last_support_point.row_index
                beds_crossed = bed_index - (last_support_point.row_index // self.row_count)
                offset = last_support_point_offset
                if beds_crossed > 0:
                    offset += beds_crossed * self.bed_spacing
                    rows_in_complete_beds = rows_since_support - (row_in_bed + 1)
                    offset += rows_in_complete_beds * self.row_spacing
                    offset += row_in_bed * self.row_spacing
                else:
                    offset += rows_since_support * self.row_spacing
            else:
                base_offset = row_in_bed * self.row_spacing
                bed_offset = bed_index * ((self.row_count - 1) * self.row_spacing + self.bed_spacing)
                offset = base_offset + bed_offset
            offset_row_coordinated = offset_curve(ab_line_cartesian, -offset).coords
            row_points = [GeoPoint.from_point(Point(x=p[0], y=p[1])) for p in offset_row_coordinated]
            row = Row(id=f'field_{self.id}_row_{i}', name=f'row_{i}',
                      points=row_points, crop=self.bed_crops[str(bed_index)])
            rows.append(row)
        return rows

    def _generate_outline(self) -> list[GeoPoint]:
        assert len(self.rows) > 0
        return self.get_buffered_area()

    def shapely_polygon(self) -> shapely.geometry.Polygon:
        return shapely.geometry.Polygon([p.tuple for p in self.outline])

    @classmethod
    def args_from_dict(cls, data: dict[str, Any]) -> dict:
        # Ensure all required fields exist with defaults
        defaults: dict[str, Any] = {
            'id': str(uuid.uuid4()),
            'name': 'Field',
            'first_row_start': None,
            'first_row_end': None,
            'row_spacing': 1,
            'row_count': 1,
            'outline_buffer_width': 1,
            'row_support_points': [],
            'bed_count': 1,
            'bed_spacing': 1,
            'bed_crops': {},
            'docking_distance': 2.0,
            'charge_dock_pose': None,
        }
        for key in defaults:
            if key in data:
                defaults[key] = data[key]
        return defaults

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        for key in ['first_row_start', 'first_row_end']:
            data[key] = GeoPoint.from_degrees(lat=data[key]['lat'],
                                              lon=data[key]['lon'] if 'lon' in data[key] else data[key]['long'])
        data['row_support_points'] = [rosys.persistence.from_dict(RowSupportPoint, sp)
                                      for sp in data.get('row_support_points', [])]
        charge_dock_tuple: tuple[float, float, float] | None = data.get('charge_dock_pose', None)
        data['charge_dock_pose'] = GeoPose.from_degrees(*charge_dock_tuple) if charge_dock_tuple else None
        return cls(**cls.args_from_dict(data))

    def get_buffered_area(self) -> list[GeoPoint]:
        outline_unbuffered: list[Point] = [
            self.first_row_end.to_local(),
            self.first_row_start.to_local(),
        ]
        if len(self.rows) > 1:
            for p in self.rows[-1].points:
                outline_unbuffered.append(p.to_local())
            outline_shape = Polygon([p.tuple for p in outline_unbuffered])
        else:
            outline_shape = LineString([p.tuple for p in outline_unbuffered])
        buffered_polygon = outline_shape.buffer(
            self.outline_buffer_width, cap_style='square', join_style='mitre', mitre_limit=math.inf)
        buffered_polygon_coords = buffered_polygon.exterior.coords
        outline = [GeoPoint.from_point(Point(x=p[0], y=p[1])) for p in buffered_polygon_coords]
        return outline

    def to_dict(self) -> dict:
        return {
            'id': self.id,
            'name': self.name,
            'first_row_start': {
                'lat': math.degrees(self.first_row_start.lat),
                'lon': math.degrees(self.first_row_start.lon),
            },
            'first_row_end': {
                'lat': math.degrees(self.first_row_end.lat),
                'lon': math.degrees(self.first_row_end.lon),
            },
            'row_spacing': self.row_spacing,
            'row_count': self.row_count,
            'outline_buffer_width': self.outline_buffer_width,
            'row_support_points': [rosys.persistence.to_dict(sp) for sp in self.row_support_points],
            'bed_count': self.bed_count,
            'bed_spacing': self.bed_spacing,
            'bed_crops': self.bed_crops,
            'docking_distance': self.docking_distance,
            'charge_dock_pose': self.charge_dock_pose.degree_tuple if self.charge_dock_pose is not None else None,
        }
