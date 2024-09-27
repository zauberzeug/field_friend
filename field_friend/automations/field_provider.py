import logging
import os
import uuid
from typing import Any, Optional

import rosys

from ..localization import GeoPoint, Gnss
from . import Field, FieldObstacle, FieldParameters


class FieldProvider(rosys.persistence.PersistentModule):
    def __init__(self, gnss: Gnss) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.gnss = gnss
        self.field_parameters_list: list[FieldParameters] = []
        self.FIELD_PARAMETERS_LIST_CHANGED = rosys.event.Event()
        """The dict of field_parameters has changed."""
        self.fields: list[Field] = []
        self.needs_backup: bool = False

        self.FIELDS_CHANGED = rosys.event.Event()
        self._init_fields()
        self.FIELD_PARAMETERS_LIST_CHANGED.register(self._init_fields)

    def backup(self) -> dict:
        return {
            'field_parameters_list': {field_parameters.id: field_parameters.to_dict() for field_parameters in self.field_parameters_list},
        }

    def restore(self, data: dict[str, dict]) -> None:
        field_parameters_dict: dict[str, dict] = data.get('field_parameters_list', {})
        field_parameters_list: list[FieldParameters] = []
        for field_parameters in field_parameters_dict.values():
            field_parameters_list.append(FieldParameters.from_dict(field_parameters))
        print(f'🔵{self.field_parameters_list}')

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELD_PARAMETERS_LIST_CHANGED.emit()

    def get_field(self, id_: str) -> Field | None:
        field_parameters = next((fp for fp in self.field_parameters_list if fp.id == id_), None)
        return field_parameters.field if field_parameters else None

    def create_field_parameters(self, new_field_parameters: FieldParameters) -> FieldParameters:
        # TODO: delete the clear when we want to save multiple fields again
        self.clear_field_parameters_list()
        self.field_parameters_list.append(new_field_parameters)
        return new_field_parameters

    def clear_field_parameters_list(self) -> None:
        self.field_parameters_list.clear()
        self.FIELD_PARAMETERS_LIST_CHANGED.emit()
        self.invalidate()

    def delete_field_parameters(self, id_: str) -> None:
        field_parameters = next((fp for fp in self.field_parameters_list if fp.id == id_), None)
        if field_parameters:
            self.field_parameters_list.remove(field_parameters)
            self.FIELD_PARAMETERS_LIST_CHANGED.emit()
            self.invalidate()

    def is_polygon(self, field: Field) -> bool:
        try:
            polygon = field.shapely_polygon
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except Exception:
            return False

    def _init_fields(self) -> None:
        field_list: list[Field] = []
        for field_parameters in self.field_parameters_list:
            field_list.append(field_parameters.field)
        self.fields = field_list
        self.FIELDS_CHANGED.emit()

    # def create_obstacle(self, field: Field, points: list[GeoPoint] = []) -> FieldObstacle:
    #     obstacle = FieldObstacle(id=f'{str(uuid.uuid4())}', name=f'obstacle_{len(field.obstacles)+1}', points=points)
    #     field.obstacles.append(obstacle)
    #     return obstacle

    # def remove_obstacle(self, field: Field, obstacle: FieldObstacle) -> None:
    #     field.obstacles.remove(obstacle)

    # # def add_obstacle_point(self, field: Field, obstacle: FieldObstacle, point: Optional[GeoPoint] = None, new_point: Optional[GeoPoint] = None) -> None:
    #     if new_point is None:
    #         assert self.gnss.current is not None
    #         positioning = self.gnss.current.location
    #         if positioning is None or positioning.lat == 0 or positioning.long == 0:
    #             rosys.notify("No GNSS position.")
    #             return
    #         if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
    #             rosys.notify("GNSS position is not accurate enough.")
    #             return
    #         new_point = positioning
    #     if point is not None:
    #         index = obstacle.points.index(point)
    #         obstacle.points[index] = new_point
    #     else:
    #         obstacle.points.append(new_point)

    # def remove_obstacle_point(self, obstacle: FieldObstacle, point: Optional[GeoPoint] = None) -> None:
    #     if obstacle.points:
    #         if point is not None:
    #             index = obstacle.points.index(point)
    #             del obstacle.points[index]
    #         else:
    #             del obstacle.points[-1]
