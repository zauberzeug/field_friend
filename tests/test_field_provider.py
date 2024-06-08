import json
from pathlib import Path

import pytest
import rosys

from field_friend.automations import FieldProvider
from field_friend.localization import GnssSimulation


def test_loading_from_old_persistence():
    field_provider = FieldProvider(GnssSimulation(rosys.driving.Odometer(rosys.hardware.WheelsSimulation())))
    field_provider.restore(json.loads(Path('tests/old_field_provider_persistence.json').read_text()))
    assert len(field_provider.fields) == 3
    field = field_provider.fields[1]
    assert len(field.points) == 4
    assert field.reference is not None
    assert field.reference.lat == pytest.approx(51.98, rel=0.01)
    assert field.reference.long == pytest.approx(7.43, rel=0.01)
    assert len(field.rows) == 2
    row = field.rows[1]
    assert len(row.points) == 2
    assert row.points[1].lat == pytest.approx(51.98, rel=0.01)
    assert row.points[1].long == pytest.approx(7.43, rel=0.01)
