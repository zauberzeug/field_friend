

import random

import numpy as np
import pytest
import rosys
from rosys.geometry import Point, Pose
from rosys.helpers import angle
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations.implements import Implement, Recorder, Tornado
from field_friend.automations.navigation import Navigation, StraightLineNavigation
from field_friend.hardware.double_wheels import WheelsSimulationWithAcceleration


@pytest.mark.parametrize('target, end_pose, max_turn_angle', [
    (Point(x=1.0, y=0.0), Pose(x=1.0, y=0.0, yaw=0.0), 1.0),
    (Point(x=1.0, y=0.005), Pose(x=1.0, y=0.005, yaw=np.deg2rad(0.29)), 1.0),
    (Point(x=1.0, y=0.01), Pose(x=1.0, y=0.01, yaw=np.deg2rad(0.58)), 1.0),
    (Point(x=1.0, y=0.1), Pose(x=1.0, y=0.018, yaw=np.deg2rad(1.0)), 1.0),
    (Point(x=1.0, y=1.0), Pose(x=1.0, y=1.0, yaw=np.deg2rad(45.0)), 45.0),
])
async def test_driving_towards_target(system: System, target: Point, end_pose: Pose, max_turn_angle: float):
    max_turn_angle = np.deg2rad(max_turn_angle)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.linear_speed_limit = 0.1
    system.automator.start(system.current_navigation.drive_towards_target(
        target, target_heading=0.0, max_turn_angle=max_turn_angle))
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped, timeout=300)
    assert system.robot_locator.pose.point.x == pytest.approx(end_pose.x, abs=0.005)
    assert system.robot_locator.pose.point.y == pytest.approx(end_pose.y, abs=0.005)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(end_pose.yaw_deg, abs=0.5)


async def test_driving_to_exact_positions(system: System):
    class Stopper(Implement):
        def __init__(self, system: System) -> None:
            super().__init__('Stopper')
            self.system = system
            self.current_stretch = 0.0
            self.workflow_started = False
            self.target_positions = [
                Point(x=0.1 + i * 0.02 + random.uniform(0, 0.005), y=0) for i in range(1, 40)
            ]
            self.current_target_position: Point | None = None
            self.new_target_position()

        async def get_move_target(self) -> Point | None:
            return self.current_target_position

        def new_target_position(self) -> None:
            if not self.target_positions:
                self.current_target_position = None
                return
            self.current_target_position = self.target_positions.pop(0)

        async def start_workflow(self) -> None:
            self.workflow_started = True
            deadline = rosys.time() + 1
            while self.workflow_started and rosys.time() < deadline:
                await rosys.sleep(0.1)
            self.workflow_started = False
            self.new_target_position()

    system.field_friend.WORK_X = 0.0
    system.current_implement = stopper = Stopper(system)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 1.0
    system.current_navigation.linear_speed_limit = 0.02  # drive really slow so we can archive the accuracy tested below
    system.automator.start()
    await forward(until=lambda: system.automator.is_running, dt=0.01)
    while stopper.target_positions:
        await forward(until=lambda: stopper.workflow_started and system.automator.is_running, dt=0.01)
        assert isinstance(stopper.current_target_position, Point)
        assert system.robot_locator.pose.point.x == pytest.approx(stopper.current_target_position.x, abs=0.001)
        assert system.robot_locator.pose.point.y == pytest.approx(stopper.current_target_position.y, abs=0.001)
        await forward(0.1)  # give robot time to update position
    system.current_navigation.linear_speed_limit = Navigation.LINEAR_SPEED_LIMIT
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.x == pytest.approx(system.current_navigation.length, abs=0.001)


@pytest.mark.parametrize('distance', (0.005, 0.01, 0.05, 0.1, 0.5, 1.0))
async def test_deceleration_different_distances(system_with_acceleration: System, distance: float):
    """Try to stop after different distances with a tolerance of 10% and a linear speed limit of 0.13m/s"""
    system = system_with_acceleration
    assert isinstance(system.field_friend.wheels, WheelsSimulationWithAcceleration)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = distance
    system.current_navigation.linear_speed_limit = 0.13
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(distance, abs=0.001)


@pytest.mark.parametrize(('linear_speed_limit', 'tolerance'), [
    (0.1, 0.0005),
    (0.13, 0.001),
    (0.2, 0.001),
    (0.3, 0.0013),
    (0.4, 0.0013),
])
async def test_deceleration_different_speeds(system_with_acceleration: System, linear_speed_limit: float, tolerance: float):
    """Try stop after 1mm with different speeds with a tolerance of 0.2mm"""
    system = system_with_acceleration
    assert isinstance(system.field_friend.wheels, WheelsSimulationWithAcceleration)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 0.005
    system.current_navigation.linear_speed_limit = linear_speed_limit
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(0.005, abs=tolerance)


@pytest.mark.parametrize('heading_degrees', (-180, -90, 0, 90, 180, 360))
async def test_driving_turn_to_yaw(system: System, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    assert system.current_navigation is not None
    system.automator.start(system.current_navigation.turn_to_yaw(heading))
    # NOTE: do not wait until automator.is_running because it will immediately stop for 0 and 360 degrees
    await forward(0.1)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.x == pytest.approx(0, abs=0.001)
    assert system.robot_locator.pose.y == pytest.approx(0, abs=0.001)
    assert angle(system.robot_locator.pose.yaw, heading) == pytest.approx(0, abs=1.0)


async def test_slippage(system: System):
    assert isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 2.0
    system.field_friend.wheels.slip_factor_right = 0.04
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert_point(system.robot_locator.pose.point, Point(x=2.0, y=0))


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
