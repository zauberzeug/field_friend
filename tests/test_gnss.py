import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.hardware.gnss import GnssSimulation
from rosys.testing import assert_point, forward

from field_friend.system import System


async def test_driving(gnss_driving: System):
    assert gnss_driving.odometer.prediction.point.x == 0
    assert gnss_driving.odometer.prediction.point.y == 0
    assert gnss_driving.gnss.last_measurement is not None
    assert gnss_driving.gnss.last_measurement.location.distance(ROBOT_GEO_START_POSITION) < 0.01
    await forward(x=2.0)
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))


async def test_connection_lost(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.mode = 'NNNN'
    gnss.is_connected = False
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.mode = 'RRRR'
    gnss.is_connected = True
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
    gnss.is_connected = False
    await forward(3)
    # robot should have stopped driving
    assert_point(gnss_driving.odometer.prediction.point, rosys.geometry.Point(x=2.0, y=0))
    gnss.is_connected = True
    gnss.mode = 'RFFF'
    await forward(5)
    # robot should continue driving
    assert gnss_driving.odometer.prediction.point.x > 2.5


async def test_device_disconnects(gnss_driving: System, gnss: GnssSimulation):
    await forward(x=2.0)
    gnss.is_connected = False
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
