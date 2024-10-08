import json
from pathlib import Path

import pytest
import rosys
from rosys.geometry import Point

from field_friend import System, localization
from field_friend.automations import FieldProvider
from field_friend.localization import GeoPoint, GnssSimulation


def test_loading_from_old_persistence():
    localization.reference = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    wheels = rosys.hardware.WheelsSimulation()
    field_provider = FieldProvider(GnssSimulation(rosys.driving.Odometer(wheels), wheels))
    field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    assert len(field_provider.fields) == 1
    field = field_provider.fields[0]
    assert field.row_number == 10
    assert field.row_spacing == 0.5
    assert field.outline_buffer_width == 2
    assert len(field.outline) == 5
    assert len(field.rows) == 10
    for row in field.rows:
        assert len(row.points) == 2
    assert field.first_row_start.lat == 51.98317071260942
    assert field.rows[0].points[0].lat == 51.98317071260942
    assert field.rows[0].points[0].long == 7.43411239981148


def test_field_outline(system: System):
    localization.reference = GeoPoint(lat=51.98317071260942, long=7.43411239981148)
    wheels = rosys.hardware.WheelsSimulation()
    field_provider = FieldProvider(GnssSimulation(rosys.driving.Odometer(wheels), wheels))
    field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    outline = field_provider.fields[0].outline
    assert len(outline) == 5  # last point = first point
