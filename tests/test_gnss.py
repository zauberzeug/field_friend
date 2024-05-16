import rosys
from rosys.testing import assert_point, forward

from field_friend.navigation.geo_point import GeoPoint
from field_friend.navigation.gnss_simulation import GnssSimulation
from field_friend.system import System


def test_distance_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
    assert point.distance(GeoPoint(lat=51.983159, long=7.434212)) == 0
    assert 6.0 < point.distance(GeoPoint(lat=51.983159, long=7.4343)) < 6.1


def test_shifted_calculation():
    point = GeoPoint(lat=51.983159, long=7.434212)
    shifted = point.shifted(rosys.geometry.Point(x=6, y=6))
    assert shifted.lat == 51.98321292429538
    assert shifted.long == 7.434299331433169


async def test_driving(gnss_driving: System):
    await forward(x=2.0)
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))


async def test_connection_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.gps_quality = 0
    await forward(1)
    # robot should have stopped driving if gnss connection is lost
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))
    gnss.gps_quality = 8
    await forward(10)
    # robot should continue driving if gnss connection is reestablished
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=4, y=0))


async def test_device_disconnects(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.disconnect()
    await forward(5)
    # robot should have stopped driving if gnss is not active anymore
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))
