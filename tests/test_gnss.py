import rosys
from rosys.testing import assert_point, forward

from field_friend.navigation.geo_point import GeoPoint
from field_friend.navigation.gnss_simulation import GnssSimulation
from field_friend.system import System


def test_distance_calculation(gnss: GnssSimulation):
    point = GeoPoint(lat=51.983159, long=7.434212)
    assert point.distance(GeoPoint(lat=51.983159, long=7.434212)) == 0
    assert 6.0 < point.distance(GeoPoint(lat=51.983159, long=7.4343)) < 6.1


async def test_driving(gnss_driving: System):
    await forward(x=2.0)
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))


async def test_device_disconnects(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.disconnect()
    await forward(5)
    # robot should have stopped driving if gnss is not active anymore
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))


async def test_connection_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.GNSS_CONNECTION_LOST.emit()
    await forward(5)
    # robot should have stopped driving if gnss connection is lost
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2, y=0))
