import json
from pathlib import Path

import pytest
from conftest import FIELD_FIRST_ROW_START
from rosys.geometry import Point

from field_friend import System
from field_friend.automations import Field, RowSupportPoint
from field_friend.localization import GeoPoint


def test_loading_from_old_persistence(system: System):
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    assert len(system.field_provider.fields) == 1
    field = system.field_provider.fields[0]
    assert field.row_number == 10
    assert field.row_spacing == 0.5
    assert field.outline_buffer_width == 2
    assert len(field.outline) == 5
    assert len(field.rows) == 10
    assert len(field.row_support_points) == 0
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
    for i, row in enumerate(field.rows):
        shift = i * field.row_spacing
        assert len(row.points) == 2
        assert row.points[0].lat == pytest.approx(field.first_row_start.lat, abs=1e-9)
        assert row.points[0].long == pytest.approx(field.first_row_start.shifted(Point(x=0, y=-shift)).long, abs=1e-9)
        assert row.points[1].lat == pytest.approx(field.first_row_end.lat, abs=1e-9)
        assert row.points[1].long == pytest.approx(field.first_row_end.shifted(Point(x=0, y=-shift)).long, abs=1e-9)


def test_add_row_support_point(system: System, field: Field):
    field_provider = system.field_provider
    field_id = field.id
    point = FIELD_FIRST_ROW_START.shifted(Point(x=0, y=-2))
    row_index = 2
    # check if distance between first and second row is correct before adding support point
    assert field.rows[0].points[0].distance(field.rows[row_index].points[0]) == pytest.approx(
        row_index * field.row_spacing, abs=1e-6)
    support_point = RowSupportPoint.from_geopoint(point, row_index)
    assert support_point.lat == point.lat
    assert support_point.long == point.long
    field_provider.add_row_support_point(field_id, support_point)
    updated_field: Field | None = field_provider.get_field(field_id)
    assert updated_field is not None
    assert len(updated_field.row_support_points) == 1
    # was the support point added correctly
    assert updated_field.row_support_points[0].row_index == row_index
    assert updated_field.row_support_points[0].lat == pytest.approx(point.lat, abs=1e-9)
    assert updated_field.row_support_points[0].long == pytest.approx(point.long, abs=1e-9)
    # check if row was moved correctly
    assert updated_field.rows[row_index].points[0].distance(point) == pytest.approx(0, abs=1e-6)
    assert updated_field.rows[row_index].points[0].lat == pytest.approx(point.lat, abs=1e-9)
    assert updated_field.rows[row_index].points[0].long == pytest.approx(point.long, abs=1e-9)
    # check if next row was moved correctly
    next_row_start_shifted = point.shifted(Point(x=0, y=-updated_field.row_spacing))
    assert updated_field.rows[row_index+1].points[0].lat == pytest.approx(next_row_start_shifted.lat, abs=1e-9)
    assert updated_field.rows[row_index+1].points[0].long == pytest.approx(next_row_start_shifted.long, abs=1e-9)


def test_add_multiple_row_support_points(system: System, field: Field):
    field_provider = system.field_provider
    field_id = field.id
    second_row_shift_point = FIELD_FIRST_ROW_START.shifted(Point(x=0, y=-0.3))
    third_row_shift_point = FIELD_FIRST_ROW_START.shifted(Point(x=0, y=-1.0))
    support_points = [
        RowSupportPoint.from_geopoint(second_row_shift_point, row_index=1),
        RowSupportPoint.from_geopoint(third_row_shift_point, row_index=2),
    ]
    for point in support_points:
        field_provider.add_row_support_point(field_id, point)
    updated_field: Field | None = field_provider.get_field(field_id)
    assert updated_field is not None
    assert len(updated_field.row_support_points) == 2
    assert [sp.row_index for sp in updated_field.row_support_points] == [1, 2]
    assert [sp.lat for sp in updated_field.row_support_points] == [
        second_row_shift_point.lat, third_row_shift_point.lat]
    assert [sp.long for sp in updated_field.row_support_points] == [
        second_row_shift_point.long, third_row_shift_point.long]
    # Check the distance between the first and second row
    assert updated_field.rows[0].points[0].distance(updated_field.rows[1].points[0]) == pytest.approx(0.3, abs=1e-6)
    assert updated_field.rows[0].points[1].distance(updated_field.rows[1].points[1]) == pytest.approx(0.3, abs=1e-6)
    # Check the distance between the first and third row
    assert updated_field.rows[0].points[0].distance(updated_field.rows[2].points[0]) == pytest.approx(1.0, abs=1e-6)
    assert updated_field.rows[0].points[1].distance(updated_field.rows[2].points[1]) == pytest.approx(1.0, abs=1e-6)
    # Check the distance between the first and fourth row
    expected_distance = 1.0 + field.row_spacing
    actual_distance_start_point = updated_field.rows[0].points[0].distance(updated_field.rows[3].points[0])
    assert actual_distance_start_point == pytest.approx(expected_distance, abs=1e-6)
    actual_distance_end_point = updated_field.rows[0].points[1].distance(updated_field.rows[3].points[1])
    assert actual_distance_end_point == pytest.approx(expected_distance, abs=1e-6)


def test_update_existing_row_support_point(system: System, field: Field):
    field_provider = system.field_provider
    field_id = field.id
    initial_shift_point = FIELD_FIRST_ROW_START.shifted(Point(x=0, y=-1.5))
    updated_shift_point = FIELD_FIRST_ROW_START.shifted(Point(x=0, y=-2.0))
    initial_point = RowSupportPoint.from_geopoint(initial_shift_point, 2)
    field_provider.add_row_support_point(field_id, initial_point)
    updated_point = RowSupportPoint.from_geopoint(updated_shift_point, 2)
    field_provider.add_row_support_point(field_id, updated_point)
    updated_field: Field | None = field_provider.get_field(field_id)
    assert updated_field is not None
    assert len(updated_field.row_support_points) == 1
    assert updated_field.row_support_points[0].row_index == 2  # 3rd row
    assert updated_field.row_support_points[0].distance(updated_shift_point) == pytest.approx(0, abs=1e-6)
    assert updated_field.rows[2].points[0].cartesian().distance(
        updated_field.rows[0].points[0].cartesian()) == pytest.approx(2.0, abs=1e-6)
    assert updated_field.rows[2].points[1].cartesian().distance(
        updated_field.rows[0].points[1].cartesian()) == pytest.approx(2.0, abs=1e-6)
    expected_distance = 2.45  # 3rd row support point + row spacing
    actual_distance = updated_field.rows[3].points[0].cartesian().distance(updated_field.rows[0].points[0].cartesian())
    assert actual_distance == pytest.approx(expected_distance, abs=1e-6)
