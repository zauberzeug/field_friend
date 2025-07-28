

import random
from typing import override

import numpy as np
import pytest
import rosys
from conftest import set_robot_pose
from rosys.geometry import Point, Pose
from rosys.helpers import angle
from rosys.testing import assert_point, forward

from field_friend import System
from field_friend.automations import AutomationWatcher
from field_friend.automations.implements import Implement, Recorder
from field_friend.automations.navigation import PathSegment, StraightLineNavigation, WaypointNavigation
from field_friend.hardware.double_wheels import WheelsSimulationWithAcceleration


async def test_driving_to_exact_positions(system: System):
    class StopperImplement(Implement):
        def __init__(self, system: System) -> None:
            super().__init__('Stopper')
            self.system = system
            self.current_stretch = 0.0
            self.workflow_started = False
            self.target_positions = [
                Point(x=0.1 + i * 0.02 + random.uniform(0, 0.005), y=0) for i in range(1, 40)
            ]
            self.current_target_position: Point | None = None
            self.pick_next_target_position()

        @override
        async def get_target(self) -> Point | None:
            return self.current_target_position

        @override
        async def start_workflow(self) -> None:
            self.workflow_started = True
            deadline = rosys.time() + 1
            while self.workflow_started and rosys.time() < deadline:
                await rosys.sleep(0.1)
            self.workflow_started = False
            self.pick_next_target_position()

        def pick_next_target_position(self) -> None:
            if not self.target_positions:
                self.current_target_position = None
                return
            self.current_target_position = self.target_positions.pop(0)

    system.field_friend.WORK_X = 0.0
    system.current_implement = stopper_implement = StopperImplement(system)
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 1.0
    system.current_navigation.linear_speed_limit = 0.02  # drive really slow so we can archive the accuracy tested below
    system.automator.start()
    await forward(until=lambda: system.automator.is_running, dt=0.01)
    while stopper_implement.target_positions:
        await forward(until=lambda: stopper_implement.workflow_started and system.automator.is_running, dt=0.01)
        assert isinstance(stopper_implement.current_target_position, Point)
        assert system.robot_locator.pose.point.x == pytest.approx(
            stopper_implement.current_target_position.x, abs=0.001)
        assert system.robot_locator.pose.point.y == pytest.approx(
            stopper_implement.current_target_position.y, abs=0.001)
        await forward(0.1)  # give robot time to update position
    system.current_navigation.linear_speed_limit = WaypointNavigation.LINEAR_SPEED_LIMIT
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


async def test_stop_when_reaching_end(system: System, detector: rosys.vision.DetectorSimulation):
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.25, y=0.0, z=0.0)))
    detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                   position=rosys.geometry.Point3d(x=0.0, y=0.0, z=0.0)))
    system.current_implement = system.implements['Weed Screw']
    system.current_navigation = system.straight_line_navigation
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = 0.5
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.001)
    assert len(detector.simulated_objects) == 1


@pytest.mark.parametrize('manual_move', (0, 0.20, 0.21))
async def test_resume_after_pause(system: System, manual_move: float):
    assert system.field_navigation is not None
    system.current_navigation = system.straight_line_navigation
    assert isinstance(system.current_navigation, StraightLineNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    system.automator.pause('test')
    await forward(until=lambda: system.automator.is_paused)
    moved_pose = system.robot_locator.pose
    moved_pose.x += manual_move
    set_robot_pose(system, moved_pose)
    system.automator.resume()
    await forward(2)
    if manual_move <= AutomationWatcher.ALLOWED_RESUME_DEVIATION:
        assert system.automator.is_running
        await forward(until=lambda: system.automator.is_stopped)
        assert system.robot_locator.pose.point.x == pytest.approx(system.straight_line_navigation.length, abs=0.1)
    else:
        assert system.automator.is_stopped


async def test_straight_path(system: System):
    system.current_navigation = system.waypoint_navigation
    assert system.current_navigation is not None
    assert isinstance(system.current_navigation.implement, Recorder)
    pose1 = Pose(x=1.0, y=0.0, yaw=0.0)
    pose2 = Pose(x=2.0, y=0.0, yaw=0.0)
    system.current_navigation.generate_path = lambda: [
        PathSegment.from_poses(system.robot_locator.pose, pose1, stop_at_end=False),
        PathSegment.from_poses(pose1, pose2),
    ]
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(pose2.x, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(pose2.y, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(pose2.yaw_deg, abs=0.1)


@pytest.mark.parametrize('start_offset', (0.5, 0.0, -0.25, -0.5, -0.75, -0.99))
async def test_start_inbetween_waypoints(system: System, start_offset: float):
    system.current_navigation = system.waypoint_navigation
    assert system.current_navigation is not None
    assert isinstance(system.current_navigation.implement, Recorder)
    # generate path which expands left and right from current pose
    start = system.robot_locator.pose.transform_pose(Pose(x=start_offset, y=0.0, yaw=0.0))
    end = start.transform_pose(Pose(x=1.0, y=0.0, yaw=0.0))
    system.current_navigation.generate_path = lambda: [PathSegment.from_poses(start, end)]
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    assert system.current_navigation.current_segment.end.x == pytest.approx(end.x, abs=0.1)
    assert system.current_navigation.current_segment.end.y == pytest.approx(end.y, abs=0.1)
    assert system.current_navigation.current_segment.end.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_start_on_end(system: System):
    system.current_navigation = system.waypoint_navigation
    assert system.current_navigation is not None
    assert isinstance(system.current_navigation.implement, Recorder)
    # set start of path 1m before current pose
    start = system.robot_locator.pose.transform_pose(Pose(x=-1, y=0.0, yaw=0.0))
    end = system.robot_locator.pose
    system.current_navigation.generate_path = lambda: [PathSegment.from_poses(start, end)]
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.current_segment is not None
    assert system.robot_locator.pose.x == pytest.approx(end.x, abs=0.1)
    assert system.robot_locator.pose.y == pytest.approx(end.y, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_skip_first_segment(system: System):
    pose1 = Pose(x=-1, y=1, yaw=-np.pi/2)
    pose2 = Pose(x=0, y=0.0, yaw=0.0)
    pose3 = Pose(x=1.0, y=1.0, yaw=np.pi/2)
    pose4 = Pose(x=0, y=2.0, yaw=np.pi)
    system.current_navigation = system.waypoint_navigation
    assert system.current_navigation is not None
    assert isinstance(system.current_navigation.implement, Recorder)

    def generate_path():
        path = [
            PathSegment.from_poses(pose1, pose2, stop_at_end=False),
            PathSegment.from_poses(pose2, pose3, stop_at_end=False),
            PathSegment.from_poses(pose3, pose4, stop_at_end=False),
            PathSegment.from_poses(pose4, pose1),
        ]
        assert system.current_navigation is not None
        path = system.current_navigation._remove_segments_behind_robot(path)  # pylint: disable=protected-access
        return path
    system.current_navigation.generate_path = generate_path
    system.automator.start()
    await forward(until=lambda: system.current_navigation is not None and system.current_navigation.current_segment is not None)
    assert system.current_navigation.current_segment is not None
    assert len(system.current_navigation.path) == 3
    assert system.current_navigation.current_segment.end.x == pytest.approx(pose3.x, abs=0.1)
    assert system.current_navigation.current_segment.end.y == pytest.approx(pose3.y, abs=0.1)
    assert system.current_navigation.current_segment.end.yaw_deg == pytest.approx(pose3.yaw_deg, abs=0.1)


@pytest.mark.parametrize('length', (1.0, 2.0))
async def test_straight_line(system: System, length: float):
    system.current_navigation = system.straight_line_navigation
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.current_navigation.length = length
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    current_segment = system.current_navigation.current_segment
    assert current_segment is not None
    assert current_segment.spline.estimated_length() == length


@pytest.mark.parametrize('heading_degrees', (-180, -90, -45, 0, 45, 90, 180, 360))
async def test_straight_line_different_headings(system: System, heading_degrees: float):
    heading = np.deg2rad(heading_degrees)
    current_pose = system.robot_locator.pose
    set_robot_pose(system, Pose(x=current_pose.x, y=current_pose.y, yaw=heading))
    assert isinstance(system.current_navigation, StraightLineNavigation)
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    current_segment = system.current_navigation.current_segment
    assert current_segment is not None
    direction = current_segment.spline.start.direction(current_segment.spline.end)
    assert angle(direction, heading) == pytest.approx(0, abs=0.1)
