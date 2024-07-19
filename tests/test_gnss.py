
from copy import deepcopy

import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.testing import assert_point, forward

from field_friend.localization import GeoPoint, GnssSimulation
from field_friend.system import System


def test_distance_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
    assert point.distance(GeoPoint(lat=51.983159, long=7.434212)) == 0
    assert 6.0 < point.distance(GeoPoint(lat=51.983159, long=7.4343)) < 6.1


def test_shifted_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
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
    gnss.gps_quality = 0
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.gps_quality = 4
    await forward(5)
    # robot should continue driving
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_rtk_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.gps_quality = 5
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.gps_quality = 4
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
    async def empty():
        return None
    await forward(x=2.0)
    gnss._create_new_record = empty  # type: ignore
    await forward(5)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))


@pytest.mark.skip('does not work anymore due to gps using wheels.pose instead of odometry.pose')
async def test_changing_reference(gnss_driving: System):
    assert gnss_driving.gnss.current is not None
    assert gnss_driving.gnss.reference is not None
    assert gnss_driving.gnss.current.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    assert gnss_driving.gnss.reference.distance(ROBOT_GEO_START_POSITION) < 0.01
    await forward(x=2.0, tolerance=0.001)
    gnss_driving.automator.pause('changing reference point')
    await forward(2)
    assert gnss_driving.gnss.current.location.distance(ROBOT_GEO_START_POSITION) == pytest.approx(2.0, 0.1)
    location = deepcopy(gnss_driving.gnss.current.location)
    gnss_driving.gnss.reference = gnss_driving.gnss.reference.shifted(rosys.geometry.Point(x=-3.0, y=0.0))
    await forward(2)
    assert gnss_driving.odometer.prediction.point.x == pytest.approx(5.0)
    assert gnss_driving.odometer.prediction.point.y == pytest.approx(0.0)
    assert location == gnss_driving.gnss.current.location
