import logging

import rosys

from ..localization import Gnss
from . import Field, RowSupportPoint


class FieldProvider(rosys.persistence.PersistentModule):
    def __init__(self, gnss: Gnss) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.gnss = gnss
        self.fields: list[Field] = []
        self.needs_backup: bool = False

        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

        self.FIELDS_CHANGED.register(self.refresh_fields)

        self.selected_field: Field | None = None
        self.FIELD_SELECTED = rosys.event.Event()
        """A field has been selected."""

    def backup(self) -> dict:
        return {
            'fields': {f.id: f.to_dict() for f in self.fields},
            'selected_field': self.selected_field.id if self.selected_field else None,
        }

    def restore(self, data: dict[str, dict]) -> None:
        fields_data: dict[str, dict] = data.get('fields', {})
        for field in list(fields_data.values()):
            new_field = Field.from_dict(field)
            self.fields.append(new_field)
        selected_field_id = data.get('selected_field')
        if selected_field_id:
            self.selected_field = self.get_field(selected_field_id)
            self.FIELD_SELECTED.emit()
        self.FIELDS_CHANGED.emit()

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()
        if self.selected_field and self.selected_field not in self.fields:
            self.selected_field = None
            self.FIELD_SELECTED.emit()

    def get_field(self, id_: str | None) -> Field | None:
        return next((f for f in self.fields if f.id == id_), None)

    def create_field(self, new_field: Field) -> Field:
        self.fields.append(new_field)
        self.invalidate()
        return new_field

    def clear_fields(self) -> None:
        self.fields.clear()
        self.invalidate()

    def delete_selected_field(self) -> None:
        if not self.selected_field:
            self.log.warning('No field selected. Nothing was deleted.')
            return
        name = self.selected_field.name
        self.fields.remove(self.selected_field)
        self.log.info('Field %s has been deleted.', name)
        self.invalidate()

    def is_polygon(self, field: Field) -> bool:
        try:
            polygon = field.shapely_polygon()
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except Exception:
            return False

    def add_row_support_point(self, field_id: str, row_support_point: RowSupportPoint) -> None:
        field = self.get_field(field_id)
        if not field:
            return
        existing_point = next((sp for sp in field.row_support_points if sp.row_index ==
                              row_support_point.row_index), None)
        if existing_point:
            field.row_support_points.remove(existing_point)
        field.row_support_points.append(row_support_point)
        self.invalidate()

    def refresh_fields(self) -> None:
        for field in self.fields:
            field.refresh()

    def select_field(self, id_: str | None) -> None:
        self.selected_field = self.get_field(id_)
        self.FIELD_SELECTED.emit()

    def update_field_parameters(self, field_id: str, name: str, row_number: int, row_spacing: float, outline_buffer_width: float) -> None:
        field = self.get_field(field_id)
        if not field:
            self.log.warning('Field with id %s not found. Cannot update parameters.', field_id)
            return
        field.name = name
        field.row_number = row_number
        field.row_spacing = row_spacing
        field.outline_buffer_width = outline_buffer_width
        self.log.info('Updated parameters for field %s: row number = %d, row spacing = %f',
                      field.name, row_number, row_spacing)
        self.invalidate()
