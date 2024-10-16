import json
from pathlib import Path

import pytest
import rosys
from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START
from rosys.geometry import Point

from field_friend import System
from field_friend.automations import Field, FieldProvider
from field_friend.localization import GeoPoint, GnssSimulation


def test_loading_from_old_persistence(system: System):
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    assert len(system.field_provider.fields) == 1
    field = system.field_provider.fields[0]
    assert field.row_number == 10
    assert field.row_spacing == 0.5
    assert field.outline_buffer_width == 2
    assert len(field.outline) == 5
    assert len(field.rows) == 10
    for row in field.rows:
        assert len(row.points) == 2
    assert field.first_row_start == FIELD_FIRST_ROW_START
    assert field.first_row_end == GeoPoint(lat=51.98334192260392, long=7.434293309874038)


def test_field_outline(system: System, field: Field):
    field = system.field_provider.fields[0]
    outline = field.outline

    def rounded_geo_point(point: GeoPoint) -> GeoPoint:
        return GeoPoint(lat=round(point.lat, 9), long=round(point.long, 9))

    def assert_in_outline(point: GeoPoint):
        assert rounded_geo_point(point) in [rounded_geo_point(p) for p in outline]

    buffer = field.outline_buffer_width
    row_offset = field.row_spacing * (field.row_number - 1)

    # Check first row boundary points
    assert_in_outline(field.first_row_start.shifted(Point(x=-buffer, y=buffer)))
    assert_in_outline(field.first_row_end.shifted(Point(x=buffer, y=buffer)))

    # Check last row boundary points
    assert_in_outline(field.first_row_start.shifted(Point(x=-buffer, y=-buffer - row_offset)))
    assert_in_outline(field.first_row_end.shifted(Point(x=buffer, y=-buffer - row_offset)))


def test_field_rows(system: System, field: Field):
    field = system.field_provider.fields[0]
    for i, r in enumerate(field.rows):
        shift = i * field.row_spacing
        assert len(r.points) == 2
        print(i)
        assert r.points[0].lat == pytest.approx(field.first_row_start.lat, 0.000_000_001)
        assert r.points[0].long == pytest.approx(field.first_row_start.shifted(Point(x=0, y=-shift)).long, 0.000_000_001)
        assert r.points[1].lat == pytest.approx(field.first_row_end.lat, 0.000_000_001)
        assert r.points[1].long == pytest.approx(field.first_row_end.shifted(Point(x=0, y=-shift)).long, 0.000_000_001)
