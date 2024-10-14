import logging

import rosys

from ..localization import Gnss
from . import Field


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

    def backup(self) -> dict:
        return {
            'fields': {f.id: f.to_dict() for f in self.fields},
        }

    def restore(self, data: dict[str, dict]) -> None:
        fields_data: dict[str, dict] = data.get('fields', {})
        for field in list(fields_data.values()):
            new_field = Field.from_dict(field)
            self.fields.append(new_field)
        self.FIELDS_CHANGED.emit()

    def invalidate(self) -> None:
        self.request_backup()
        self.FIELDS_CHANGED.emit()

    def get_field(self, id_: str | None) -> Field | None:
        return next((f for f in self.fields if f.id == id_), None)

    def create_field(self, new_field: Field) -> Field:
        # TODO: delete the clear when we want to save multiple fields again
        self.fields.clear()
        self.fields.append(new_field)
        self.invalidate()
        return new_field

    def clear_fields(self) -> None:
        self.fields.clear()
        self.invalidate()

    def delete_field(self, id_: str) -> None:
        field = self.get_field(id_)
        if field:
            self.fields.remove(field)
            self.invalidate()

    def is_polygon(self, field: Field) -> bool:
        try:
            polygon = field.shapely_polygon()
            return polygon.is_valid and polygon.geom_type == 'Polygon'
        except Exception:
            return False

    def refresh_fields(self) -> None:
        for field in self.fields:
            field.refresh()
