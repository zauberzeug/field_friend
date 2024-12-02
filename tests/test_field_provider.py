import json
import uuid
from pathlib import Path

import pytest
from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START
from rosys.geometry import Point

from field_friend import System
from field_friend.automations import Field, RowSupportPoint
from field_friend.localization import GeoPoint


def test_loading_from_persistence(system: System):
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    assert len(system.field_provider.fields) == 1
    field = system.field_provider.fields[0]
    assert field.row_count == 10
    assert field.row_spacing == 0.5
    assert field.outline_buffer_width == 2
    assert len(field.outline) == 5
    assert len(field.rows) == 10
    assert len(field.row_support_points) == 0
    for row in field.rows:
        assert len(row.points) == 2
    assert field.first_row_start == FIELD_FIRST_ROW_START
    assert field.first_row_end == GeoPoint(lat=51.98334192260392, long=7.434293309874038)


def test_loading_from_persistence_with_errors(system: System):
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence_with_errors.json').read_text()))
    assert len(system.field_provider.fields) == 1
    field = system.field_provider.fields[0]
    # should set outline_buffer_width to default value because it is missing in the persistence data
    assert field.outline_buffer_width == 1
    # should not write the not_existing_value to the field
    assert not hasattr(field, 'not_existing_value')
    assert field.row_count == 10
    assert field.row_spacing == 0.5
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
    row_offset = field.row_spacing * (field.row_count - 1)

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
        row_index * field.row_spacing, abs=1e-8)
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
    assert updated_field.rows[row_index].points[0].distance(point) == pytest.approx(0, abs=1e-8)
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
    assert updated_field.rows[0].points[0].distance(updated_field.rows[1].points[0]) == pytest.approx(0.3, abs=1e-8)
    assert updated_field.rows[0].points[1].distance(updated_field.rows[1].points[1]) == pytest.approx(0.3, abs=1e-8)
    # Check the distance between the first and third row
    assert updated_field.rows[0].points[0].distance(updated_field.rows[2].points[0]) == pytest.approx(1.0, abs=1e-8)
    assert updated_field.rows[0].points[1].distance(updated_field.rows[2].points[1]) == pytest.approx(1.0, abs=1e-8)
    # Check the distance between the first and fourth row
    expected_distance = 1.0 + field.row_spacing
    actual_distance_start_point = updated_field.rows[0].points[0].distance(updated_field.rows[3].points[0])
    assert actual_distance_start_point == pytest.approx(expected_distance, abs=1e-8)
    actual_distance_end_point = updated_field.rows[0].points[1].distance(updated_field.rows[3].points[1])
    assert actual_distance_end_point == pytest.approx(expected_distance, abs=1e-8)


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
    assert updated_field.row_support_points[0].distance(updated_shift_point) == pytest.approx(0, abs=1e-8)
    assert updated_field.rows[2].points[0].cartesian().distance(
        updated_field.rows[0].points[0].cartesian()) == pytest.approx(2.0, abs=1e-8)
    assert updated_field.rows[2].points[1].cartesian().distance(
        updated_field.rows[0].points[1].cartesian()) == pytest.approx(2.0, abs=1e-8)
    expected_distance = 2.45  # 3rd row support point + row spacing
    actual_distance = updated_field.rows[3].points[0].cartesian().distance(updated_field.rows[0].points[0].cartesian())
    assert actual_distance == pytest.approx(expected_distance, abs=1e-8)


def test_create_multiple_fields(system: System):
    field_provider = system.field_provider
    field_provider.clear_fields()
    assert len(field_provider.fields) == 0

    # Create first field
    field1 = Field(
        id=str(uuid.uuid4()),
        name='Field 1',
        first_row_start=FIELD_FIRST_ROW_START,
        first_row_end=FIELD_FIRST_ROW_END,
        row_count=5,
        row_spacing=0.5
    )
    created_field1 = field_provider.create_field(field1)
    assert len(field_provider.fields) == 1
    assert field_provider.get_field(created_field1.id) == created_field1

    # Create second field
    field2 = Field(
        id=str(uuid.uuid4()),
        name='Field 2',
        first_row_start=FIELD_FIRST_ROW_START.shifted(Point(x=10, y=10)),
        first_row_end=FIELD_FIRST_ROW_END.shifted(Point(x=10, y=10)),
        row_count=3,
        row_spacing=0.75
    )
    created_field2 = field_provider.create_field(field2)
    assert len(field_provider.fields) == 2
    assert field_provider.get_field(created_field2.id) == created_field2

    assert field_provider.fields[0].name == 'Field 1'
    assert field_provider.fields[0].row_count == 5
    assert field_provider.fields[0].row_spacing == 0.5
    assert field_provider.fields[0].bed_count == 1
    assert field_provider.fields[0].bed_spacing == 0.5

    assert field_provider.fields[1].name == 'Field 2'
    assert field_provider.fields[1].row_count == 3
    assert field_provider.fields[1].row_spacing == 0.75
    assert field_provider.fields[1].bed_count == 1
    assert field_provider.fields[1].bed_spacing == 0.5

    assert field_provider.fields[0].id != field_provider.fields[1].id


def test_select_field(system: System, field: Field):
    field_provider = system.field_provider
    assert field_provider.selected_field.id == field.id
    field_provider.select_field(field.id)
    assert field_provider.selected_field is not None
    assert field_provider.selected_field.id == field.id


def test_delete_selected_field(system: System, field: Field):
    field_provider = system.field_provider
    field_provider.select_field(field.id)
    field_provider.delete_selected_field()
    assert len(field_provider.fields) == 0
    assert field_provider.get_field(field.id) is None


def test_create_field_with_multiple_beds(system: System, field: Field):
    field_provider = system.field_provider
    row_spacing = 0.5
    bed_spacing = 1.5
    row_count = 3
    bed_count = 2

    field_with_beds = Field(
        id=str(uuid.uuid4()),
        name='Multi-bed Field',
        first_row_start=FIELD_FIRST_ROW_START,
        first_row_end=FIELD_FIRST_ROW_END,
        row_count=row_count,
        row_spacing=row_spacing,
        bed_count=bed_count,
        bed_spacing=bed_spacing
    )
    created_field = field_provider.create_field(field_with_beds)
    assert len(field_provider.fields) == 2
    assert field_provider.get_field(created_field.id) == created_field
    assert created_field.bed_count == bed_count
    assert created_field.bed_spacing == bed_spacing
    assert created_field.row_count == row_count
    assert created_field.row_spacing == row_spacing
    # Test first bed rows
    for i in range(row_count):
        expected_start = field_with_beds.first_row_start.shifted(Point(x=0, y=-i*row_spacing))
        expected_end = field_with_beds.first_row_end.shifted(Point(x=0, y=-i*row_spacing))
        assert created_field.rows[i].points[0].lat == pytest.approx(expected_start.lat, abs=1e-8)
        assert created_field.rows[i].points[0].long == pytest.approx(expected_start.long, abs=1e-8)
        assert created_field.rows[i].points[1].lat == pytest.approx(expected_end.lat, abs=1e-8)
        assert created_field.rows[i].points[1].long == pytest.approx(expected_end.long, abs=1e-8)
    assert created_field.rows[2].points[0].lat == pytest.approx(FIELD_FIRST_ROW_START.lat, abs=1e-8)
    # Test second bed rows
    for i in range(row_count):
        row_index = i + row_count
        y_offset = -((row_index-1)*row_spacing + bed_spacing)
        expected_start = field_with_beds.first_row_start.shifted(Point(x=0, y=y_offset))
        expected_end = field_with_beds.first_row_end.shifted(Point(x=0, y=y_offset))
        assert created_field.rows[row_index].points[0].lat == pytest.approx(expected_start.lat, abs=1e-8)
        assert created_field.rows[row_index].points[0].long == pytest.approx(expected_start.long, abs=1e-8)
        assert created_field.rows[row_index].points[1].lat == pytest.approx(expected_end.lat, abs=1e-8)
        assert created_field.rows[row_index].points[1].long == pytest.approx(expected_end.long, abs=1e-8)
