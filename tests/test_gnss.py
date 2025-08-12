import numpy as np
import pytest
import rosys
from conftest import ROBOT_GEO_START_POSITION
from rosys.geometry import Pose
from rosys.hardware import GnssSimulation, ImuSimulation
from rosys.hardware.gnss import GpsQuality
from rosys.testing import assert_point, forward

from field_friend.system import System


async def test_driving(gnss_driving: System):
    assert_point(gnss_driving.robot_locator.pose.point, rosys.geometry.Point(x=0, y=0))
    assert gnss_driving.gnss is not None
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


@pytest.mark.parametrize('roll_direction', (-1, 1))
@pytest.mark.parametrize('pitch_direction', (-1, 1))
async def test_height_correction(system: System, imu: ImuSimulation, roll_direction: int, pitch_direction: int):
    # pylint: disable=protected-access
    imu.roll = np.deg2rad(10.0) * roll_direction
    imu.pitch = np.deg2rad(10.0) * pitch_direction
    await forward(1)
    corrected_pose = system.robot_locator._correct_gnss_with_imu(Pose(x=0.0, y=0.0, yaw=0.0))
    assert corrected_pose.x == pytest.approx(pitch_direction * -0.108, abs=0.01)
    assert corrected_pose.y == pytest.approx(roll_direction * 0.112, abs=0.01)
