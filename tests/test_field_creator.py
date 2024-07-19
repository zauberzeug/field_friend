import pytest
import rosys
from rosys.geometry import Point

from field_friend import System
from field_friend.automations import Row
from field_friend.interface.components.field_creator import FieldCreator
from field_friend.localization import GeoPoint


def test_geometry_computation(system: System):
    field_creator = FieldCreator(system)
    field_creator.first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    field_creator.first_row_end = field_creator.first_row_start.shifted(Point(x=0, y=10))
    field_creator.last_row_end = field_creator.first_row_start.shifted(Point(x=10, y=10))
    assert field_creator.build_geometry()
    assert len(field_creator.field.rows) == 20
    outline = field_creator.field.outline
    assert len(outline) == 4
    assert outline[0].x == pytest.approx(-field_creator.padding)
    assert outline[0].y == pytest.approx(-field_creator.padding_bottom)
    assert outline[1].x == pytest.approx(-field_creator.padding)
    assert outline[1].y == pytest.approx(10 + field_creator.padding)
    assert outline[2].x == pytest.approx(10 + field_creator.padding)
    assert outline[2].y == pytest.approx(10 + field_creator.padding)
    assert outline[3].x == pytest.approx(10 + field_creator.padding)
    assert outline[3].y == pytest.approx(-field_creator.padding_bottom)


def test_wrong_row_spacing(system: System):
    field_creator = FieldCreator(system)
    field_creator.first_row_start = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    field_creator.first_row_end = field_creator.first_row_start.shifted(Point(x=0, y=10))
    field_creator.last_row_end = field_creator.first_row_start.shifted(Point(x=10, y=10))
    field_creator.row_spacing = 45
    assert not field_creator.build_geometry()
    assert len(field_creator.field.rows) == 1, 'the row should still be there even if the spacing is wrong'
