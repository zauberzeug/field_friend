import math
import uuid
from dataclasses import dataclass, field
from typing import Any, Self

import rosys
from rosys.geometry import GeoPoint


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
