from dataclasses import dataclass, field
from statistics import mean
from typing import Any, Literal, Optional, TypedDict, Union

import rosys
from geographiclib.geodesic import Geodesic
from rosys.geometry import Point
from shapely.geometry import LineString, Polygon

from field_friend.navigation.point_transformation import wgs84_to_cartesian

from .plant_provider import Plant


@dataclass(slots=True, kw_only=True)
class FieldObstacle:
    id: str
    name: str
    points_wgs84: list[list] = field(default_factory=list)

    def points(self, reference_point: list) -> list[Point]:
        if len(self.points_wgs84) > 0:
            cartesian_points = []
            for point in self.points_wgs84:
                cartesian_point = wgs84_to_cartesian([reference_point[0], reference_point[1]], point)
                cartesian_points.append(Point(x=cartesian_point[0], y=cartesian_point[1]))
            return cartesian_points
        else:
            return []


@dataclass(slots=True, kw_only=True)
class Row:
    id: str
    name: str
    points_wgs84: list[list] = field(default_factory=list)
    reverse: bool = False
    crops: list[Plant] = field(default_factory=list)

    def points(self, reference_point: list) -> list[Point]:
        if len(self.points_wgs84) > 0:
            cartesian_points = []
            for point in self.points_wgs84:
                cartesian_point = wgs84_to_cartesian([reference_point[0], reference_point[1]], point)
                cartesian_points.append(Point(x=cartesian_point[0], y=cartesian_point[1]))
            return cartesian_points
        else:
            return []

    def reversed(self):
        return Row(
            id=self.id,
            name=self.name,
            points_wgs84=list(reversed(self.points_wgs84)),
        )

    def clear_crops(self):
        self.crops.clear()


@dataclass(slots=True, kw_only=True)
class Field:
    id: str
    name: str
    outline_wgs84: list[list] = field(default_factory=list)
    reference_lat: Optional[float] = None
    reference_lon: Optional[float] = None
    visualized: bool = False
    obstacles: list[FieldObstacle] = field(default_factory=list)
    rows: list[Row] = field(default_factory=list)

    @property
    def reference(self) -> list:
        return [self.reference_lat, self.reference_lon]

    @property
    def outline(self) -> list[Point]:
        if len(self.outline_wgs84) > 0:
            cartesian_outline = []
            for point in self.outline_wgs84:
                cartesian_point = wgs84_to_cartesian([self.reference_lat, self.reference_lon], point)
                cartesian_outline.append(Point(x=cartesian_point[0], y=cartesian_point[1]))
            return cartesian_outline
        else:
            return []

    def area(self) -> float:
        if len(self.outline) > 0:
            polygon = Polygon([(p.x, p.y) for p in self.outline])
            return polygon.area
        else:
            return 0.0

    def worked_area(self, worked_rows: int) -> float:
        worked_area = 0.0
        if self.area() > 0:
            worked_area = worked_rows * self.area() / len(self.rows)
        return worked_area


class Active_object(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Union[Row, FieldObstacle]


class FieldProvider(rosys.persistence.PersistentModule):
    def __init__(self) -> None:
        super().__init__()
        self.fields: list[Field] = []
        self.active_field: Optional[Field] = None
        self.active_object: Optional[Active_object] = None

        self.FIELD_SELECTED = rosys.event.Event()
        """The currently selected field has changed"""

        self.OBJECT_SELECTED = rosys.event.Event()
        """a row or obstacle has been selected or deselected."""

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
        self.active_field = None
        self.active_object = None
        self.OBJECT_SELECTED.emit()
        self.invalidate()

    def clear_fields(self) -> None:
        self.fields.clear()
        self.active_field = None
        self.active_object = None
        self.OBJECT_SELECTED.emit()
        self.invalidate()

    def set_reference(self, field: Field, point: list) -> None:
        if field.reference_lat is None:
            field.reference_lat = point[0]
        if field.reference_lon is None:
            field.reference_lon = point[1]
        self.invalidate()

    def add_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.append(obstacle)
        self.invalidate()

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.remove(obstacle)
        self.active_object = None
        self.OBJECT_SELECTED.emit()
        self.invalidate()

    def add_row(self, field: Field, row: Row) -> None:
        field.rows.append(row)
        self.invalidate()

    def remove_row(self, field: Field, row: Row) -> None:
        field.rows.remove(row)
        self.active_object = None
        self.OBJECT_SELECTED.emit()
        self.invalidate()

    def select_field(self, field: Optional[Field] = None) -> None:
        self.active_field = field
        self.FIELD_SELECTED.emit()
        self.active_object = None
        self.OBJECT_SELECTED.emit()

    def select_object(self, object_id: Optional[str] = None, object_type: Optional[Literal["Obstacles", "Rows", "Outline"]] = None) -> None:
        if self.active_field is not None and object_id is not None and object_type is not None:
            if object_type == "Obstacles":
                for obstacle in self.active_field.obstacles:
                    if object_id == obstacle.id:
                        self.active_object = {'object_type': object_type, 'object': obstacle}
            elif object_type == "Rows":
                for row in self.active_field.rows:
                    if object_id == row.id:
                        self.active_object = {'object_type': object_type, 'object': row}
            else:
                self.active_object = None
        else:
            self.active_object = None
        self.OBJECT_SELECTED.emit()

    def is_polygon(self, field: Field) -> bool:
        try:
            polygon = Polygon(field.outline_wgs84)
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except:
            return False

    # TODO currently the function does only sort even fields where rows have the same length
    # the function need to be extended for more special cases

    def sort_rows(self, field: Field) -> None:
        if len(field.rows) <= 1:
            rosys.notify(f'There are not enough rows that can be sorted.', type='warning')
            return

        for row in field.rows:
            if len(row.points_wgs84) < 1:
                rosys.notify(f'Row {row.name} has to few points. Sorting not possible.', type='warning')
                return

        def get_centroid(row: Row) -> Point:
            polyline = LineString(row.points_wgs84)
            return polyline.centroid

        reference_centroid = get_centroid(field.rows[0])

        def get_distance(row: Row, direction: str):
            row_centroid = get_centroid(row)
            distance = Geodesic.WGS84.Inverse(reference_centroid.x, reference_centroid.y,
                                              row_centroid.x, row_centroid.y)["s12"]
            if direction == "x" and reference_centroid.x > row_centroid.x:
                distance *= -1
            elif direction == "y" and reference_centroid.y > row_centroid.y:
                distance *= -1
            print(f'{row.name} has a dist of {distance}')

            return distance

        field_index = self.fields.index(field)
        rows_axis = []

        for row in field.rows:
            direction = Geodesic.WGS84.Inverse(
                row.points_wgs84[0][0], row.points_wgs84[0][1], row.points_wgs84[-1][0], row.points_wgs84[-1][1])['azi1']
            if direction < 0:
                direction = Geodesic.WGS84.Inverse(
                    row.points_wgs84[-1][0], row.points_wgs84[-1][1], row.points_wgs84[0][0], row.points_wgs84[0][1])['azi1']
            rows_axis.append(direction)
        sort_axis = mean(rows_axis)  # mean direction of all rows

        direction = "x" if abs(reference_centroid.x - field.rows[-1].points_wgs84[0][0]) > abs(
            reference_centroid.y - field.rows[-1].points_wgs84[0][1]) else "y"
        self.fields[field_index].rows = sorted(field.rows, key=lambda row: get_distance(row, direction=direction))
        self.FIELDS_CHANGED.emit()
