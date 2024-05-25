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
    assert isinstance(system.straight_line_navigation.tool, Recorder)
    system.automator.start(system.straight_line_navigation.start())
    await forward(2)
    assert system.automator.is_running
    await forward(20)
    assert not system.automator.is_running, 'default is 2m, so automation should stop'
    assert system.odometer.prediction.point.x == pytest.approx(2.0, abs=0.1)
