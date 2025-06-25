import math
from dataclasses import dataclass

import rosys
import shapely
from rosys.geometry import GeoPoint, Point
from shapely import offset_curve
from shapely.geometry import LineString, Polygon

from field_friend.automations.field_description import FieldDescription


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
