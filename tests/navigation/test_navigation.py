

import numpy as np
import pytest
import rosys
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations.implements import Recorder, Tornado
from field_friend.automations.navigation import StraightLineNavigation


# TODO: check path not driving
async def test_straight_line(system: System):
    assert_point(system.robot_locator.pose.point, rosys.geometry.Point(x=0, y=0))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)

# TODO: check path not driving or remove


async def test_straight_line_with_tornado(system_with_tornado: System):
    system = system_with_tornado
    assert_point(system.robot_locator.pose.point, rosys.geometry.Point(x=0, y=0))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_implement = system.implements['Tornado']
    assert isinstance(system.current_navigation.implement, Tornado)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert not system.automator.is_running, 'automation should stop after default length'
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    assert system.automator.is_stopped


# TODO: check path not driving
async def test_straight_line_with_high_angles(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    predicted_yaw = 190
    start_yaw = system.robot_locator.pose.yaw
    target_yaw = start_yaw + np.deg2rad(predicted_yaw)
    await system.driver.wheels.drive(*system.driver._throttle(0, 0.1))  # pylint: disable=protected-access
    await forward(until=lambda: abs(rosys.helpers.angle(system.robot_locator.pose.yaw, target_yaw)) < np.deg2rad(0.1))
    await system.driver.wheels.stop()
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.length = 1.0
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=24)
    assert system.robot_locator.pose.point.x == pytest.approx(-0.985, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(-0.174, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(predicted_yaw, abs=5)
