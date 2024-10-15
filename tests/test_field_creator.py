from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START

from field_friend import System
from field_friend.interface.components.field_creator import FieldCreator


def test_field_creation(system: System, field_creator: FieldCreator):
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert system.field_provider.fields[0].name == 'Field 1'
    assert system.field_provider.fields[0].row_spacing == 0.5
    assert system.field_provider.fields[0].row_number == 10
    assert system.field_provider.fields[0].first_row_start == FIELD_FIRST_ROW_START
    assert system.field_provider.fields[0].first_row_end == FIELD_FIRST_ROW_END
    assert len(system.field_provider.fields[0].rows) == 10


def test_field_creation_wrong_row_spacing(system: System, field_creator: FieldCreator):
    field_creator.row_spacing = 0
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert len(system.field_provider.fields[0].rows) == 10, 'the row should still be there even if the spacing is wrong'
