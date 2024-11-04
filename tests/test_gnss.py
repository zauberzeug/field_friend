

from copy import deepcopy

import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import assert_point, forward

from field_friend import localization
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.localization import GeoPoint, GnssSimulation
from field_friend.system import System


def test_distance_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
    assert point.distance(GeoPoint(lat=51.983159, long=7.434212)) == 0
    assert 6.0 < point.distance(GeoPoint(lat=51.983159, long=7.4343)) < 6.1


def test_shifted_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
    localization.reference = deepcopy(point)
    shifted = point.shifted(rosys.geometry.Point(x=6, y=6))
    # coordinates should be x pointing north, y pointing west (verified with https://www.meridianoutpost.com/resources/etools/calculators/calculator-latitude-longitude-distance.php?)
    assert shifted.lat == pytest.approx(51.983212924295)
    assert shifted.long == pytest.approx(7.434124668461)
    assert_point(shifted.cartesian(), rosys.geometry.Point(x=6, y=6))


async def test_driving(gnss_driving: System):
    assert gnss_driving.odometer.prediction.point.x == 0
    assert gnss_driving.odometer.prediction.point.y == 0
    assert gnss_driving.gnss.current is not None
    assert gnss_driving.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    await forward(x=2.0)
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))


async def test_connection_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.mode = 'NNNN'
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.mode = 'RRRR'
    await forward(5)
    # robot should continue driving
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_simulated_rtk(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.mode = 'SSSS'
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    await forward(5)
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_rtk_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.mode = 'FFFF'
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.mode = 'RFFF'
    await forward(5)
    # robot should continue driving
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_device_disconnects(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.disconnect()
    await forward(5)
    # robot should have stopped driving
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_record_is_none(gnss_driving: System, gnss: GnssSimulation):
    # pylint: disable=protected-access
    async def empty():
        return None
    await forward(x=2.0)
    gnss._create_new_record = empty  # type: ignore
    await forward(5)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))


async def test_only_update_when_standing(gnss_driving: System, gnss: GnssSimulation):
    # pylint: disable=protected-access
    assert isinstance(gnss_driving.current_navigation, StraightLineNavigation)
    gnss_driving.current_navigation.length = 10.0
    assert isinstance(gnss_driving.current_navigation.implement, Recorder)
    assert gnss._last_gnss_pose.x == pytest.approx(gnss._last_odometer_pose.x)
    gnss_driving.automator.start()
    await forward(until=lambda: gnss_driving.automator.is_running)
    await forward(x=5.0)
    assert gnss._last_gnss_pose.x != pytest.approx(gnss._last_odometer_pose.x)
    await forward(until=lambda: gnss_driving.automator.is_stopped)
    await forward(2.0)
    assert gnss._last_gnss_pose.x == pytest.approx(gnss._last_odometer_pose.x)
