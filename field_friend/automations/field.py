import math
import uuid
from dataclasses import dataclass, field
from typing import Any, Self

import rosys
import shapely
from rosys.geometry import GeoPoint, Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon


def _extract_coords_from_geometry(geom) -> list[tuple[float, float]]:
    if geom.geom_type == 'LineString':
        return list(geom.coords)
    elif geom.geom_type in ('MultiLineString', 'GeometryCollection'):
        # NOTE: MultiLineString requires concatenating segments while avoiding duplicate points
        coords = []
        for line in geom.geoms:
            line_coords = list(line.coords)
            if coords and line_coords:
                if coords[-1] == line_coords[0]:
                    line_coords = line_coords[1:]
            coords.extend(line_coords)
        return coords
    elif geom.geom_type == 'Point':
        return [(float(geom.x), float(geom.y))]
    else:
        try:
            return list(geom.coords)
        except (AttributeError, NotImplementedError):
            return []


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
    waypoint_index: int

    @classmethod
    def from_geopoint(cls, geopoint: GeoPoint, row_index: int, waypoint_index: int) -> Self:
        return cls(lat=geopoint.lat, lon=geopoint.lon, row_index=row_index, waypoint_index=waypoint_index)


@dataclass(slots=True, kw_only=True)
class FieldDescription:
    id: str
    name: str
    first_row_start: GeoPoint
    first_row_end: GeoPoint
    row_spacing: float = 0.5
    row_count: int = 10
    outline_buffer_width: float = 2
    row_support_points: list[RowSupportPoint] = field(default_factory=list)
    bed_count: int = 1
    bed_spacing: float = 0.5
    bed_crops: dict[str, str | None] = field(default_factory=dict)

    def __post_init__(self):
        if self.bed_spacing is None:
            self.bed_spacing = self.row_spacing * 2
        if not self.bed_crops:
            self.bed_crops = {str(i): None for i in range(self.bed_count)}
        self.row_support_points.sort(key=lambda sp: sp.row_index)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
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
            'bed_crops': {}
        }
        for key in defaults:
            if key in data:
                defaults[key] = data[key]

        for key in ['first_row_start', 'first_row_end']:
            point_data = defaults[key]
            if isinstance(point_data, dict):
                point_data = dict(point_data)  # Type cast for linter
                defaults[key] = GeoPoint.from_degrees(lat=point_data['lat'],
                                                      lon=point_data['lon'] if 'lon' in point_data else point_data['long'])
        defaults['row_support_points'] = [rosys.persistence.from_dict(RowSupportPoint, sp)
                                          for sp in defaults.get('row_support_points', [])]
        return cls(**defaults)

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
            'bed_crops': self.bed_crops
        }


class ComputedField:
    def __init__(self, field_description: FieldDescription):
        self.source = field_description
        self.rows = self._generate_rows()
        self.outline = self._generate_outline()

    def __eq__(self, other) -> bool:
        if not isinstance(other, ComputedField):
            return False
        return self.source.id == other.source.id

    @property
    def outline_cartesian(self) -> list[rosys.geometry.Point]:
        return [p.to_local() for p in self.outline]

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
            total_rows = self.source.row_count * self.source.bed_count
            worked_area = worked_rows * self.area() / total_rows
        return worked_area

    def shapely_polygon(self) -> shapely.geometry.Polygon:
        return shapely.geometry.Polygon([p.tuple for p in self.outline])

    def _generate_rows(self) -> list[Row]:
        assert self.source.first_row_start is not None
        assert self.source.first_row_end is not None

        rows: list[Row] = []
        total_rows = self.source.row_count * self.source.bed_count

        waypoints_by_row = {}
        for sp in self.source.row_support_points:
            if sp.row_index not in waypoints_by_row:
                waypoints_by_row[sp.row_index] = []
            waypoints_by_row[sp.row_index].append(sp)

        for row_waypoints in waypoints_by_row.values():
            row_waypoints.sort(key=lambda wp: wp.waypoint_index)

        ab_line_cartesian = LineString([self.source.first_row_start.to_local().tuple,
                                       self.source.first_row_end.to_local().tuple])

        current_reference_shape = ab_line_cartesian
        last_shape_change_row = -1

        for i in range(total_rows):
            bed_index = i // self.source.row_count
            row_in_bed = i % self.source.row_count

            base_offset = row_in_bed * self.source.row_spacing
            bed_offset = bed_index * ((self.source.row_count - 1) *
                                      self.source.row_spacing + self.source.bed_spacing)
            total_offset = base_offset + bed_offset

            row_waypoints = waypoints_by_row.get(i, [])

            if row_waypoints:
                if i == 0:
                    # NOTE: Row 0 treats all waypoints as curve points regardless of waypoint_index
                    row_start = self.source.first_row_start
                    row_end = self.source.first_row_end

                    row_points = [row_start]
                    row_points.extend(row_waypoints)
                    row_points.append(row_end)

                    current_reference_shape = LineString([p.to_local().tuple for p in row_points])
                    last_shape_change_row = i
                else:
                    spacing_waypoint = next((wp for wp in row_waypoints if wp.waypoint_index == 0), None)
                    curve_waypoints = [wp for wp in row_waypoints if wp.waypoint_index > 0]

                    if spacing_waypoint:
                        spacing_distance = ab_line_cartesian.distance(shapely.geometry.Point(
                            [spacing_waypoint.to_local().x, spacing_waypoint.to_local().y]))
                        total_offset = spacing_distance

                    if curve_waypoints:
                        if spacing_waypoint:
                            offset_line = offset_curve(ab_line_cartesian, -total_offset)
                            offset_coords = _extract_coords_from_geometry(offset_line)
                            row_start = GeoPoint.from_point(Point(x=offset_coords[0][0], y=offset_coords[0][1]))
                            row_end = GeoPoint.from_point(Point(x=offset_coords[-1][0], y=offset_coords[-1][1]))
                        else:
                            if last_shape_change_row >= 0:
                                rows_since_change = i - last_shape_change_row
                                relative_offset = rows_since_change * self.source.row_spacing
                                offset_line = offset_curve(current_reference_shape, -relative_offset)
                            else:
                                offset_line = offset_curve(ab_line_cartesian, -total_offset)

                            offset_coords = _extract_coords_from_geometry(offset_line)
                            row_start = GeoPoint.from_point(Point(x=offset_coords[0][0], y=offset_coords[0][1]))
                            row_end = GeoPoint.from_point(Point(x=offset_coords[-1][0], y=offset_coords[-1][1]))

                        row_points = [row_start]
                        row_points.extend(curve_waypoints)
                        row_points.append(row_end)

                        current_reference_shape = LineString([p.to_local().tuple for p in row_points])
                        last_shape_change_row = i

                    else:
                        offset_line = offset_curve(ab_line_cartesian, -total_offset)
                        offset_coords = _extract_coords_from_geometry(offset_line)
                        row_points = [GeoPoint.from_point(Point(x=p[0], y=p[1])) for p in offset_coords]

                        current_reference_shape = offset_line
                        last_shape_change_row = i
            else:
                if last_shape_change_row >= 0:
                    rows_since_change = i - last_shape_change_row
                    relative_offset = rows_since_change * self.source.row_spacing
                    offset_line = offset_curve(current_reference_shape, -relative_offset)
                else:
                    offset_line = offset_curve(ab_line_cartesian, -total_offset)

                offset_coords = _extract_coords_from_geometry(offset_line)
                row_points = [GeoPoint.from_point(Point(x=p[0], y=p[1])) for p in offset_coords]

            row = Row(id=f'field_{self.source.id}_row_{i}', name=f'row_{i}',
                      points=row_points, crop=self.source.bed_crops[str(bed_index)])
            rows.append(row)

        return rows

    def _generate_outline(self) -> list[GeoPoint]:
        assert len(self.rows) > 0
        return self.get_buffered_area()

    def get_buffered_area(self) -> list[GeoPoint]:
        outline_unbuffered: list[Point] = [
            self.source.first_row_end.to_local(),
            self.source.first_row_start.to_local(),
        ]
        if len(self.rows) > 1:
            for p in self.rows[-1].points:
                outline_unbuffered.append(p.to_local())
            outline_shape = Polygon([p.tuple for p in outline_unbuffered])
        else:
            outline_shape = LineString([p.tuple for p in outline_unbuffered])
        buffered_polygon = outline_shape.buffer(
            self.source.outline_buffer_width, cap_style='square', join_style='mitre', mitre_limit=math.inf)
        buffered_polygon_coords = buffered_polygon.exterior.coords
        outline = [GeoPoint.from_point(Point(x=p[0], y=p[1])) for p in buffered_polygon_coords]
        return outline
