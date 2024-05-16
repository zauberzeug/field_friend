import rosys
from rosys.testing import assert_point, forward

from field_friend.navigation import GNSSRecord, GnssSimulation
from field_friend.navigation.geo_point import GeoPoint
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


def test_updating_record():
    pose_location: rosys.geometry.Pose | None = None

    def located(pose: rosys.geometry.Pose):
        nonlocal pose_location
        pose_location = pose
        ic(pose_location)
    odometer = rosys.driving.Odometer(rosys.hardware.)
    gnss = GnssSimulation(odometer)
    gnss.ROBOT_POSE_LOCATED.register(located)
    record = GNSSRecord()
    record.timestamp = 0
    record.latitude = 51.983159
    record.longitude = 7.434212
    record.mode = 'simulation'
    record.gps_qual = 4
    assert gnss.reference is None
    gnss._update_record(record)  # pylint: disable=protected-access
    assert pose_location == rosys.geometry.Pose()
    assert gnss.current.timestamp == 0
    assert gnss.current.has_location
    assert gnss.reference is not None

    record.timestamp = 1
    record.gps_qual = 4
    record.longitude = 7.4343
    gnss._update_record(record)  # pylint: disable=protected-access
    assert gnss.current.timestamp == 1
    assert pose_location == rosys.geometry.Pose(x=6, time=1)


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


async def test_rtk_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.gps_quality = 4
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
