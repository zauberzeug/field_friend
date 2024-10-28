import pytest
from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START
from rosys.geometry import Point

from field_friend import System, localization
from field_friend.automations.field import Field
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.interface.components.support_point_dialog import SupportPointDialog


def test_field_creation(system: System, field_creator: FieldCreator):
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert system.field_provider.fields[0].name == 'Field'
    assert system.field_provider.fields[0].row_spacing == 0.5
    assert system.field_provider.fields[0].row_count == 10
    assert system.field_provider.fields[0].first_row_start == FIELD_FIRST_ROW_START
    assert system.field_provider.fields[0].first_row_end == FIELD_FIRST_ROW_END
    assert len(system.field_provider.fields[0].rows) == 10


def test_field_creation_wrong_row_spacing(system: System, field_creator: FieldCreator):
    field_creator.row_spacing = 0
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert len(system.field_provider.fields[0].rows) == 10, 'the row should still be there even if the spacing is wrong'


def test_support_point_dialog(system: System, field: Field):
    dialog = SupportPointDialog(system)
    row_index = 2
    dialog.row_name = row_index+1
    test_location = FIELD_FIRST_ROW_START.shifted(point=Point(x=0, y=-1.5))
    system.gnss.current.location = test_location
    dialog.next()
    dialog.next()
    assert system.field_provider.fields[0].row_support_points[0].cartesian(
    ).x - FIELD_FIRST_ROW_START.cartesian().x == pytest.approx(0, abs=0.001)
    assert system.field_provider.fields[0].row_support_points[0].cartesian(
    ).y - FIELD_FIRST_ROW_START.cartesian().y == pytest.approx(-1.5, abs=0.001)
    assert system.field_provider.fields[0].row_support_points[0].row_index == row_index
    assert dialog.support_point_coordinates == test_location
