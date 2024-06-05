import logging
import uuid
from typing import Any, Literal, Optional, TypedDict, Union

import rosys
from geographiclib.geodesic import Geodesic
from rosys.geometry import Point

from ..localization import GeoPoint, Gnss
from . import Field, FieldObstacle, Row


class Active_object(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Union[Row, FieldObstacle]


class FieldProvider(rosys.persistence.PersistentModule):

    def __init__(self, gnss: Gnss) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.gnss = gnss
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

    def get_field(self, id_: str | None) -> Field | None:
        for field in self.fields:
            if field.id == id_:
                return field
        return None

    def backup(self) -> dict:
        return {
            'fields': rosys.persistence.to_dict(self.fields),
            'active_field': None if self.active_field is None else self.active_field.id,
        }

    def restore(self, data: dict[str, Any]) -> None:
        fields_data = data.get('fields', [])
        rosys.persistence.replace_list(self.fields, Field, fields_data)

        # NOTE we had some changes in persistence; this code transforms old to new format
        for i, f in enumerate(self.fields):
            outline = fields_data[i].get('outline_wgs84', [])
            for coords in outline:
                f.points.append(GeoPoint(lat=coords[0], long=coords[1]))
            rlat = fields_data[i].get('reference_lat', None)
            rlong = fields_data[i].get('reference_lon', None)
            if rlat is not None and rlong is not None:
                f.reference = GeoPoint(lat=rlat, long=rlong)
            rows = fields_data[i].get('rows', [])
            for j, row in enumerate(rows):
                for point in row.get('points_wgs84', []):
                    f.rows[j].points.append(GeoPoint(lat=point[0], long=point[1]))

        self.active_field = next((f for f in self.fields if f.id == data.get('active_field')), None)

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()

    def create_field(self, points: list[GeoPoint] = []) -> Field:
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'field_{len(self.fields)+1}', points=points)
        if points:
            self.set_reference(field, points[0])
        self.fields.append(field)
        self.invalidate()
        return field

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

    def set_reference(self, field: Field, point: GeoPoint) -> None:
        if field.reference is None:
            field.reference = point

    def create_obstacle(self, field: Field, points: list[GeoPoint] = []) -> FieldObstacle:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'obstacle_{len(field.obstacles)+1}', points=points)
        field.obstacles.append(obstacle)
        self.invalidate()
        return obstacle

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.remove(obstacle)
        self.active_object = None
        self.OBJECT_SELECTED.emit()
        self.invalidate()

    def create_row(self, field: Field, points: list[GeoPoint] = []) -> Row:
        assert not points or len(points) == 2
        row = Row(id=f'{str(uuid.uuid4())}', name=f'row_{len(field.rows)+1}', points=points)
        field.rows.append(row)
        self.invalidate()
        return row

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
        self.invalidate()

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
            polygon = field.shapely_polygon
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except Exception:
            return False

    def sort_rows(self, field: Field) -> None:
        # TODO currently the function does only sort even fields where rows have the same length
        # the function need to be extended for more special cases
        if len(field.rows) <= 1:
            rosys.notify(f'There are not enough rows that can be sorted.', type='warning')
            return

        for row in field.rows:
            if len(row.points) < 1:
                rosys.notify(f'Row {row.name} has to few points. Sorting not possible.', type='warning')
                return

        def get_centroid(row: Row) -> Point:
            return row.shapely_line.centroid

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
        p = field.rows[-1].points[0]
        direction = "x" if abs(reference_centroid.x - p.lat) > abs(reference_centroid.y - p.long) else "y"
        self.fields[field_index].rows = sorted(field.rows, key=lambda row: get_distance(row, direction=direction))
        self.FIELDS_CHANGED.emit()

    def ensure_field_reference(self, field: Field) -> None:
        if self.gnss.device is None:
            self.log.warning('not creating Reference because no GNSS device found')
            rosys.notify('No GNSS device found', 'negative')
            return
        if self.gnss.current is None or self.gnss.current.gps_qual != 4:
            self.log.warning('not creating Reference because no RTK fix available')
            rosys.notify('No RTK fix available', 'negative')
            return
        if field.reference is None:
            ref = self.gnss.reference
            if ref is None:
                self.log.warning('not creating Point because no reference position available')
                rosys.notify('No reference position available')
                return
            field.reference = ref
        if self.gnss.reference != field.reference:
            self.gnss.reference = field.reference

    async def add_field_point(self, field: Field, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        assert self.gnss.current is not None
        positioning = self.gnss.current.location
        if positioning is None or positioning.lat == 0 or positioning.long == 0:
            rosys.notify("No GNSS position.")
            return
        if self.gnss.current.gps_qual != 4:
            rosys.notify("GNSS position is not accurate enough.")
            return
        new_point = positioning
        if point is not None:
            index = field.points.index(point)
            if index == 0:
                self.set_reference(field, new_point)
            field.points[index] = new_point
        else:
            if len(field.points) < 1:
                self.set_reference(field, new_point)
                self.gnss.reference = new_point
            field.points.append(new_point)
        self.invalidate()

    def remove_field_point(self, field: Field, point: Optional[GeoPoint] = None) -> None:
        if point is not None:
            index = field.points.index(point)
            del field.points[index]
        elif field.points:
            del field.points[-1]
        self.invalidate()

    async def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if self.gnss.current.gps_qual != 4:
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if self.gnss.device != 'simulation':
            self.ensure_field_reference(field)
        if point is not None:
            index = obstacle.points.index(point)
            obstacle.points[index] = new_point
        else:
            obstacle.points.append(new_point)
        assert self.active_object
        self.select_object(self.active_object['object'].id, 'Obstacles')
        self.invalidate()

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[GeoPoint] = None) -> None:
        if obstacle.points:
            if point is not None:
                index = obstacle.points.index(point)
                del obstacle.points[index]
            else:
                del obstacle.points[-1]
            assert self.active_object
            self.select_object(self.active_object['object'].id, 'Obstacles')
            self.invalidate()

    def add_row_point(self, field: Field, row: Row, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if self.gnss.current.gps_qual != 4:
                rosys.notify("GNSS position is not accurate enough.")
                return
            new_point = positioning
        if self.gnss.device != 'simulation':
            self.ensure_field_reference(field)
        if point is not None:
            index = row.points.index(point)
            row.points[index] = new_point
        else:
            row.points.append(new_point)
        assert self.active_object
        self.select_object(self.active_object['object'].id, 'Rows')
        self.invalidate()

    def remove_row_point(self, row: Row, point: Optional[GeoPoint] = None) -> None:
        if row.points:
            if point is not None:
                index = row.points.index(point)
                del row.points[index]
            else:
                del row.points[-1]
            assert self.active_object
            self.select_object(self.active_object['object'].id, 'Rows')
            self.invalidate()

    def move_row(self, field: Field, row: Row, next: bool = False) -> None:
        index = field.rows.index(row)
        if next:
            if index == len(field.rows)-1:
                field.rows[index], field.rows[0] = field.rows[0], field.rows[index]
            else:
                field.rows[index], field.rows[index+1] = field.rows[index+1], field.rows[index]
        else:
            field.rows[index], field.rows[index-1] = field.rows[index-1], field.rows[index]
        self.invalidate()
