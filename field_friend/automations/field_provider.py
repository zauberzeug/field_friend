import logging
from typing import Any

import rosys
from rosys.event import Event

from .computed_field import ComputedField, Row
from .field_description import FieldDescription, RowSupportPoint


class FieldProvider(rosys.persistence.Persistable):
    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.field_provider')
        self.field_descriptions: list[FieldDescription] = []
        self.needs_backup: bool = False

        self.FIELDS_CHANGED: Event = Event()
        """The dict of fields has changed."""

        self.selected_field_id: str | None = None
        self.FIELD_SELECTED: Event = Event()
        """A field has been selected."""

        self.only_specific_beds: bool = False
        self._selected_beds: list[int] = []

    @property
    def selected_beds(self) -> list[int]:
        return self._selected_beds

    @selected_beds.setter
    def selected_beds(self, value: list[int]) -> None:
        self._selected_beds = sorted(value)

    @property
    def selected_field(self) -> ComputedField | None:
        if self.selected_field_id is None:
            return None
        return self.get_field(self.selected_field_id)

    @property
    def fields(self) -> list[ComputedField]:
        return [self.get_field(field_description.id) for field_description in self.field_descriptions if self.get_field(field_description.id) is not None]

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'field_descriptions': {field_description.id: field_description.to_dict() for field_description in self.field_descriptions},
            'selected_field_id': self.selected_field_id
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        # Handle backward compatibility: old format with 'fields' key
        if 'fields' in data:
            fields_data: dict[str, dict] = data.get('fields', {})
            for field_dict in fields_data.values():
                field_description = FieldDescription.from_dict(field_dict)
                self.field_descriptions.append(field_description)
            self.selected_field_id = data.get('selected_field')
        else:
            # New format with 'field_descriptions' key
            descriptions_data: dict[str, dict] = data.get('field_descriptions', {})
            for desc_dict in descriptions_data.values():
                field_description = FieldDescription.from_dict(desc_dict)
                self.field_descriptions.append(field_description)
            self.selected_field_id = data.get('selected_field_id')

        self.FIELDS_CHANGED.emit()

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()
        if self.selected_field_id and not self.get_field_description(self.selected_field_id):
            self.selected_field_id = None
            self.only_specific_beds = False
            self.clear_selected_beds()
            self.FIELD_SELECTED.emit()

    def get_field_description(self, field_id: str) -> FieldDescription | None:
        return next((field_description for field_description in self.field_descriptions if field_description.id == field_id), None)

    def get_field(self, field_id: str) -> ComputedField | None:
        field_description = self.get_field_description(field_id)
        if field_description is None:
            return None
        return ComputedField(field_description)

    def add_field_description(self, field_description: FieldDescription) -> ComputedField:
        self.field_descriptions.append(field_description)
        self.select_field(field_description.id)
        self.invalidate()
        return self.get_field(field_description.id)

    def clear_fields(self) -> None:
        self.field_descriptions.clear()
        self.invalidate()

    def delete_selected_field(self) -> None:
        if not self.selected_field_id:
            self.log.warning('No field selected. Nothing was deleted.')
            return
        field_description = self.get_field_description(self.selected_field_id)
        if field_description:
            name = field_description.name
            self.field_descriptions.remove(field_description)
            self.log.info('Field %s has been deleted.', name)
            self.invalidate()

    def add_row_support_point(self, field_id: str, row_support_point: RowSupportPoint) -> None:
        field_description = self.get_field_description(field_id)
        if not field_description:
            return
        existing_point = next((sp for sp in field_description.row_support_points
                               if sp.row_index == row_support_point.row_index
                               and sp.waypoint_index == row_support_point.waypoint_index), None)
        if existing_point:
            field_description.row_support_points.remove(existing_point)

        field_description.row_support_points.append(row_support_point)
        self.invalidate()

    def select_field(self, field_id: str | None) -> None:
        self.selected_field_id = field_id
        self.clear_selected_beds()
        self.FIELD_SELECTED.emit()
        self.request_backup()

    def update_field_parameters(self, *,
                                field_id: str,
                                name: str,
                                row_count: int,
                                row_spacing: float,
                                outline_buffer_width: float,
                                bed_count: int,
                                bed_spacing: float,
                                bed_crops: dict[str, str | None]) -> None:
        field_description = self.get_field_description(field_id)
        if not field_description:
            self.log.warning('Field with id %s not found. Cannot update parameters.', field_id)
            return
        field_description.name = name
        field_description.row_count = row_count
        field_description.row_spacing = row_spacing
        field_description.bed_count = bed_count
        field_description.bed_spacing = bed_spacing
        field_description.outline_buffer_width = outline_buffer_width
        bed_crops = bed_crops.copy()
        if len(bed_crops) < bed_count:
            for i in range(bed_count - len(bed_crops)):
                bed_crops[str(i+len(field_description.bed_crops))] = None
            field_description.bed_crops = bed_crops
        elif len(bed_crops) > bed_count:
            for i in range(len(bed_crops) - bed_count):
                bed_crops.pop(str(bed_count + i))
            field_description.bed_crops = bed_crops
        else:
            field_description.bed_crops = bed_crops
        self.log.info('Updated parameters for field %s: row number = %d, row spacing = %f',
                      field_description.name, row_count, row_spacing)
        self.invalidate()

    def clear_selected_beds(self) -> None:
        self.only_specific_beds = False
        self.selected_beds = []

    def get_rows_to_work_on(self) -> list[Row]:
        selected_field = self.selected_field
        if not selected_field:
            self.log.warning('No field selected. Cannot get rows to work on.')
            return []
        if selected_field.source.bed_count == 1:
            return selected_field.rows
        if not self.only_specific_beds:
            return selected_field.rows
        if len(self.selected_beds) == 0:
            self.log.warning('No beds selected. Cannot get rows to work on.')
            return []
        row_indices = []
        for bed in self.selected_beds:
            for row_index in range(selected_field.source.row_count):
                row_indices.append(bed * selected_field.source.row_count + row_index)
        rows_to_work_on = [row for i, row in enumerate(selected_field.rows) if i in row_indices]
        return rows_to_work_on

    def is_row_in_selected_beds(self, row_index: int) -> bool:
        if not self.only_specific_beds:
            return True
        selected_field = self.selected_field
        if selected_field is None:
            return False
        bed_index = row_index // selected_field.source.row_count
        return bed_index in self.selected_beds
