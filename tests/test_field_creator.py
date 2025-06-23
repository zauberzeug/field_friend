import pytest
from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START
from rosys.geometry import GeoPose

from field_friend import System
from field_friend.automations.field import ComputedField
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.interface.components.support_point_dialog import SupportPointDialog


def test_field_creation(system: System, field_creator: FieldCreator):
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert system.field_provider.fields[0].source.name == 'Field'
    assert system.field_provider.fields[0].source.row_spacing == 0.5
    assert system.field_provider.fields[0].source.row_count == 10
    assert system.field_provider.fields[0].source.first_row_start == FIELD_FIRST_ROW_START
    assert system.field_provider.fields[0].source.first_row_end == FIELD_FIRST_ROW_END
    assert len(system.field_provider.fields[0].rows) == 10 * 2
    assert system.field_provider.fields[0].source.bed_count == 2
    assert system.field_provider.fields[0].source.bed_spacing == 1.0
    assert system.field_provider.fields[0].source.bed_crops == {'0': 'sugar_beet', '1': 'garlic'}


def test_field_creation_wrong_row_spacing(system: System, field_creator: FieldCreator):
    field_creator.row_spacing = 0
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert len(system.field_provider.fields[0].rows) == 10 * \
        2, 'the row should still be there even if the spacing is wrong'


def test_support_point_dialog(system: System, field: ComputedField):
    dialog = SupportPointDialog(system)
    row_index = 2
    dialog.row_name = row_index
    test_location = FIELD_FIRST_ROW_START.shift_by(x=0, y=-1.5)
    system.gnss.last_measurement.pose = GeoPose(lat=test_location.lat, lon=test_location.lon, heading=0)
    dialog.next()
    dialog.next()
    row_support_point_cartesian = system.field_provider.fields[0].source.row_support_points[0].to_local()
    row_start_cartesian = FIELD_FIRST_ROW_START.to_local()
    assert row_support_point_cartesian.x - row_start_cartesian.x == pytest.approx(0, abs=0.001)
    assert row_support_point_cartesian.y - row_start_cartesian.y == pytest.approx(-1.5, abs=0.001)
    assert system.field_provider.fields[0].source.row_support_points[0].row_index == row_index
    assert dialog.support_point_coordinates == test_location
