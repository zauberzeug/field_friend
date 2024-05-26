import pytest
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import Field
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.automations.tool import Recorder
from field_friend.navigation import GeoPoint, GnssSimulation


async def test_straight_line(system: System):
    assert system.odometer.prediction.point.x == 0
    assert isinstance(system.straight_line_navigation, StraightLineNavigation)
    assert isinstance(system.straight_line_navigation.implement, Recorder)
    system.automator.start(system.straight_line_navigation.start())
    await forward(2)
    assert system.automator.is_running
    await forward(20)
    assert not system.automator.is_running, 'default is 2m, so automation should stop'
    assert system.odometer.prediction.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)


# async def test_start_weeding_auto_selects_rows(system: System, field: Field, gnss: GnssSimulation):
#     system.automator.start(system.straight_line_navigation.start())
#     await forward(1)
#     assert system.automator.is_running
#     assert system.weeding.start_row_id == field.rows[0].id
#     assert system.weeding.end_row_id == field.rows[-1].id


# async def test_weeding_after_modifying_rows(system: System, field: Field, gnss: GnssSimulation):
#     system.automator.start(system.tools['weeding']())
#     await forward(1)
#     system.automator.stop('change row')
#     await forward(1)
#     system.field_provider.remove_row(field, field.rows[0])
#     system.field_provider.create_row(field, points=[GeoPoint(lat=51.98318416921418, long=7.4342004020500285),
#                                                     GeoPoint(lat=51.98312378543273, long=7.434291470886676)])
#     system.automator.start(system.tools['weeding']())
#     await forward(1)
#     assert system.automator.is_running
