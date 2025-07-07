import rosys
from rosys.hardware.gnss import GnssSimulation, GpsQuality
from rosys.testing import assert_point, forward

from field_friend.system import System

from .conftest import ROBOT_GEO_START_POSITION


async def test_driving(gnss_driving: System):
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=0, y=0))
    assert gnss_driving.gnss.last_measurement is not None
    assert gnss_driving.gnss.last_measurement.point.distance(ROBOT_GEO_START_POSITION) < 0.01
    await forward(x=2.0)
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=2.0, y=0))


async def test_connection_lost(gnss_driving: System, gnss: GnssSimulation):
    # pylint: disable=protected-access
    await forward(x=2.0)
    gnss._is_connected = False
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=2.0, y=0))
    gnss._is_connected = True
    await forward(5)
    # robot should continue driving
    assert gnss_driving.robot_locator.pose.point.x > 2.5


async def test_simulated_rtk(gnss_driving: System, gnss: GnssSimulation):
    # pylint: disable=protected-access
    await forward(x=2.0)
    gnss._gps_quality = GpsQuality.RTK_FIXED
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=2.0, y=0))
    await forward(5)
    assert gnss_driving.robot_locator.pose.point.x > 2.5


async def test_rtk_lost(gnss_driving: System, gnss: GnssSimulation):
    # pylint: disable=protected-access
    await forward(x=2.0)
    gnss._gps_quality = GpsQuality.PPS
    gnss._is_connected = False
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=2.0, y=0))
    gnss._is_connected = True
    gnss._gps_quality = GpsQuality.RTK_FIXED
    await forward(5)
    # robot should continue driving
    assert gnss_driving.robot_locator.pose.point.x > 2.5
