import json
from pathlib import Path

import pytest
import rosys
from conftest import FIELD_FIRST_ROW_END, FIELD_FIRST_ROW_START
from rosys.geometry import Point

from field_friend import System
from field_friend.automations import FieldProvider
from field_friend.localization import GnssSimulation


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
    assert field.first_row_end == FIELD_FIRST_ROW_END


def test_field_outline(system: System):
    wheels = rosys.hardware.WheelsSimulation()
    system.field_provider = FieldProvider(GnssSimulation(rosys.driving.Odometer(wheels), wheels))
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    outline = system.field_provider.fields[0].outline
    assert len(outline) == 5  # last point = first point
    # TODO calculate the points here of the boundary


def test_field_rows(system: System):
    system.field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    field = system.field_provider.fields[0]
    assert field.rows[0].points[0].lat == pytest.approx(field.first_row_start.lat, abs=0.00000001)
    assert field.rows[0].points[0].long == pytest.approx(field.first_row_start.long, abs=0.00000001)
    assert field.rows[0].points[1].lat == pytest.approx(field.first_row_end.lat, abs=0.00000001)
    assert field.rows[0].points[1].long == pytest.approx(field.first_row_end.long, abs=0.00000001)
    # check if the second row is correctly shifted
    assert field.rows[1].points[0].lat == pytest.approx(51.98333344134316, 0.000000001)
    assert field.rows[1].points[0].long == pytest.approx(7.43424369675644, 0.000000001)
    assert field.rows[1].points[1].lat == pytest.approx(51.98333746581215, 0.000000001)
    assert field.rows[1].points[1].long == pytest.approx(7.43429424063123, 0.000000001)
    # check if the last row is correctly shifted
    assert field.rows[9].points[0].lat == pytest.approx(51.98329778701178, 0.000000001)
    assert field.rows[9].points[0].long == pytest.approx(7.434251142846739, 0.000000001)
    assert field.rows[9].points[1].lat == pytest.approx(51.98330181147759, 0.000000001)
    assert field.rows[9].points[1].long == pytest.approx(7.434301686682065, 0.000000001)
