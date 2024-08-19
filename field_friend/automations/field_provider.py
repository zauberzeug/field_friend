import logging
import os
import uuid
from typing import Any, Optional

import rosys
from geographiclib.geodesic import Geodesic
from rosys.geometry import Point

from .. import localization
from ..localization import GeoPoint, Gnss
from . import Field, FieldObstacle, Row


class FieldProvider(rosys.persistence.PersistentModule):

    def __init__(self, gnss: Gnss) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.gnss = gnss
        self.fields: list[Field] = []

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
        }

    def restore(self, data: dict[str, Any]) -> None:
        fields_data = data.get('fields', [])
        rosys.persistence.replace_list(self.fields, Field, fields_data)

        # NOTE we had some changes in persistence; this code transforms old to new format
        for i, f in enumerate(self.fields):
            outline = fields_data[i].get('outline_wgs84', [])
            for coords in outline:
                f.points.append(GeoPoint(lat=coords[0], long=coords[1]))
            rows = fields_data[i].get('rows', [])
            for j, row in enumerate(rows):
                for point in row.get('points_wgs84', []):
                    f.rows[j].points.append(GeoPoint(lat=point[0], long=point[1]))

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()

    def create_field(self, points: list[GeoPoint] = []) -> Field:
        new_id = str(uuid.uuid4())
        field = Field(id=f'{new_id}', name=f'field_{len(self.fields)+1}', points=points)
        self.fields.append(field)
        self.invalidate()
        return field

    def remove_field(self, field: Field) -> None:
        self.fields.remove(field)
        self.FIELDS_CHANGED.emit()
        self.invalidate()

    def clear_fields(self) -> None:
        self.fields.clear()
        self.FIELDS_CHANGED.emit()
        self.invalidate()

    def update_reference(self) -> None:
        if self.gnss.current is None:
            rosys.notify('No GNSS position available.')
            return
        localization.reference = self.gnss.current.location
        os.utime('main.py')

    def create_obstacle(self, field: Field, points: list[GeoPoint] = []) -> FieldObstacle:
        obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'obstacle_{len(field.obstacles)+1}', points=points)
        field.obstacles.append(obstacle)
        self.invalidate()
        return obstacle

    def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
        field.obstacles.remove(obstacle)
        self.invalidate()

    def create_row(self, field: Field, points: list[GeoPoint] = []) -> Row:
        row = Row(id=f'{str(uuid.uuid4())}', name=f'row_{len(field.rows)+1}', points=points)
        field.rows.append(row)
        self.invalidate()
        return row

    def remove_row(self, field: Field, row: Row) -> None:
        field.rows.remove(row)
        self.invalidate()

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

    async def add_field_point(self, field: Field, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        assert self.gnss.current is not None
        positioning = self.gnss.current.location
        if positioning is None or positioning.lat == 0 or positioning.long == 0:
            rosys.notify("No GNSS position.")
            return
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            rosys.notify("GNSS position is not accurate enough.")
            return
        new_point = positioning
        if point is not None:
            index = field.points.index(point)
            field.points[index] = new_point
        else:
            field.points.append(new_point)
        self.invalidate()

    def remove_field_point(self, field: Field, point: Optional[GeoPoint] = None) -> None:
        if point is not None:
            index = field.points.index(point)
            del field.points[index]
        elif field.points:
            del field.points[-1]
        self.invalidate()

    def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
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
        self.invalidate()

    def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[GeoPoint] = None) -> None:
        if obstacle.points:
            if point is not None:
                index = obstacle.points.index(point)
                del obstacle.points[index]
            else:
                del obstacle.points[-1]
            self.invalidate()

    def add_row_point(self, field: Field, row: Row, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
        if new_point is None:
            assert self.gnss.current is not None
            positioning = self.gnss.current.location
            if positioning is None or positioning.lat == 0 or positioning.long == 0:
                rosys.notify("No GNSS position.")
                return
            if not("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
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
        self.invalidate()

    def remove_row_point(self, row: Row, point: Optional[GeoPoint] = None) -> None:
        if row.points:
            if point is not None:
                index = row.points.index(point)
                del row.points[index]
            else:
                del row.points[-1]
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
