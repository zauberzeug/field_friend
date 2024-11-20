import logging
from typing import Any

import rosys

from .field import Field, Row, RowSupportPoint


class FieldProvider(rosys.persistence.PersistentModule):
    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.fields: list[Field] = []
        self.needs_backup: bool = False

        self.FIELDS_CHANGED = rosys.event.Event()
        """The dict of fields has changed."""

        self.selected_field: Field | None = None
        self.FIELD_SELECTED = rosys.event.Event()
        """A field has been selected."""

        self.FIELDS_CHANGED.register(self.refresh_fields)
        self.FIELD_SELECTED.register(self.clear_selected_beds)

        self._only_specific_beds: bool = False
        self._selected_beds: list[int] = []

    @property
    def selected_beds(self) -> list[int]:
        return self._selected_beds

    @selected_beds.setter
    def selected_beds(self, value: list[int]) -> None:
        self._selected_beds = sorted(value)

    def backup(self) -> dict:
        return {
            'fields': {f.id: f.to_dict() for f in self.fields},
            'selected_field': self.selected_field.id if self.selected_field else None,
        }

    def restore(self, data: dict[str, Any]) -> None:
        fields_data: dict[str, dict] = data.get('fields', {})
        for field in list(fields_data.values()):
            new_field = Field.from_dict(field)
            self.fields.append(new_field)
        selected_field_id: str | None = data.get('selected_field')
        if selected_field_id:
            self.selected_field = self.get_field(selected_field_id)
            self.FIELD_SELECTED.emit()
        self.FIELDS_CHANGED.emit()

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()
        if self.selected_field and self.selected_field not in self.fields:
            self.selected_field = None
            self._only_specific_beds = False
            self.selected_beds = []
            self.FIELD_SELECTED.emit()

    def get_field(self, id_: str | None) -> Field | None:
        return next((f for f in self.fields if f.id == id_), None)

    def create_field(self, new_field: Field) -> Field:
        self.fields.append(new_field)
        self.select_field(new_field.id)
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

    def update_field_parameters(self, *,
                                field_id: str,
                                name: str,
                                row_count: int,
                                row_spacing: float,
                                outline_buffer_width: float,
                                bed_count: int,
                                bed_spacing: float) -> None:
        field = self.get_field(field_id)
        if not field:
            self.log.warning('Field with id %s not found. Cannot update parameters.', field_id)
            return
        field.name = name
        field.row_count = row_count
        field.row_spacing = row_spacing
        field.bed_count = bed_count
        field.bed_spacing = bed_spacing
        field.outline_buffer_width = outline_buffer_width
        self.log.info('Updated parameters for field %s: row number = %d, row spacing = %f',
                      field.name, row_count, row_spacing)
        self.invalidate()

    def clear_selected_beds(self) -> None:
        self._only_specific_beds = False
        self.selected_beds = []

    def get_rows_to_work_on(self) -> list[Row]:
        if not self.selected_field:
            self.log.warning('No field selected. Cannot get rows to work on.')
            return []
        if self.selected_field.bed_count == 1:
            return self.selected_field.rows
        if not self._only_specific_beds:
            return self.selected_field.rows
        if len(self.selected_beds) == 0:
            self.log.warning('No beds selected. Cannot get rows to work on.')
            return []
        row_indices = []
        for bed in self.selected_beds:
            for row_index in range(self.selected_field.row_count):
                row_indices.append((bed - 1) * self.selected_field.row_count + row_index)
        return [row for i, row in enumerate(self.selected_field.rows) if i in row_indices]
