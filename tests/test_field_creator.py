import rosys

from field_friend import System
from field_friend.automations import Row
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.localization import GeoPoint


def test_geometry_computation(system: System):
    field_creator = FieldCreator(system)
    field_creator.first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    field_creator.first_row_end = field_creator.first_row_start.shifted(rosys.geometry.Point(x=0, y=10))
    field_creator.last_row_end = field_creator.first_row_start.shifted(rosys.geometry.Point(x=10, y=10))
    assert field_creator.field.reference is None
    assert field_creator.build_geometry()
    assert len(field_creator.field.rows) == 20
    assert field_creator.field.reference == field_creator.first_row_start


def test_wrong_row_spacing(system: System):
    field_creator = FieldCreator(system)
    field_creator.first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    field_creator.first_row_end = field_creator.first_row_start.shifted(rosys.geometry.Point(x=0, y=10))
    field_creator.last_row_end = field_creator.first_row_start.shifted(rosys.geometry.Point(x=10, y=10))
    field_creator.row_spacing = 45
    assert not field_creator.build_geometry()
    assert len(field_creator.field.rows) == 0
