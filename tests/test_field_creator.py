from rosys.geometry import Point

from field_friend import System, localization
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.localization import GeoPoint


def test_geometry_computation(system: System):
    field_creator = FieldCreator(system)
    localization.reference = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    first_row_end = first_row_start.shifted(Point(x=0, y=10))
    field_creator.first_row_start = first_row_start
    field_creator.row_spacing = 0.5
    field_creator.row_number = 10
    field_creator.confirm_geometry()
    field_creator.first_row_end = first_row_end
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert system.field_provider.fields[0].name == 'Field 1'
    assert system.field_provider.fields[0].row_spacing == 0.5
    assert system.field_provider.fields[0].row_number == 10
    assert system.field_provider.fields[0].first_row_start == first_row_start
    assert system.field_provider.fields[0].first_row_end == first_row_end
    assert len(system.field_provider.fields[0].rows) == 11


def test_wrong_row_spacing(system: System):
    field_creator = FieldCreator(system)
    localization.reference = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    first_row_end = first_row_start.shifted(Point(x=0, y=10))
    field_creator.first_row_start = first_row_start
    field_creator.row_spacing = 0.5
    field_creator.row_number = 10
    field_creator.confirm_geometry()
    field_creator.first_row_end = first_row_end
    assert len(system.field_provider.fields) == 0
    field_creator.next()
    assert len(system.field_provider.fields) == 1
    assert len(system.field_provider.fields[0].rows) == 10, 'the row should still be there even if the spacing is wrong'
