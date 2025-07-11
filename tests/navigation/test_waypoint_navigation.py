import numpy as np
import pytest
from rosys.geometry import Pose
from rosys.testing import forward

from field_friend import System
from field_friend.automations.implements import Recorder
from field_friend.automations.navigation import PathSegment, WaypointNavigation


async def test_straight_path(system: System):
    pose1 = Pose(x=1.0, y=0.0, yaw=0.0)
    pose2 = Pose(x=2.0, y=0.0, yaw=0.0)

    def generate_path():
        return [
            PathSegment.from_poses(system.robot_locator.pose, pose1, stop_at_end=False),
            PathSegment.from_poses(pose1, pose2),
        ]
    system.current_navigation = system.waypoint_navigation
    assert isinstance(system.current_navigation, WaypointNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.generate_path = generate_path  # type: ignore
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    await forward(until=lambda: system.automator.is_stopped)
    assert system.robot_locator.pose.point.x == pytest.approx(pose2.x, abs=0.1)
    assert system.robot_locator.pose.point.y == pytest.approx(pose2.y, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(pose2.yaw_deg, abs=0.1)


@pytest.mark.parametrize('start_offset', (0.5, 0.0, -0.25, -0.5, -0.75, -0.99))
async def test_start_inbetween_waypoints(system: System, start_offset: float):
    start = system.robot_locator.pose.transform_pose(Pose(x=start_offset, y=0.0, yaw=0.0))
    end = start.transform_pose(Pose(x=1.0, y=0.0, yaw=0.0))

    def generate_path():
        return [
            PathSegment.from_poses(start, end),
        ]
    system.current_navigation = system.waypoint_navigation
    assert isinstance(system.current_navigation, WaypointNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.generate_path = generate_path  # type: ignore
    system.automator.start()
    await forward(until=lambda: system.automator.is_running)
    assert system.current_navigation.target is not None
    assert system.current_navigation.target.point.x == pytest.approx(end.x, abs=0.1)
    assert system.current_navigation.target.point.y == pytest.approx(end.y, abs=0.1)
    assert system.current_navigation.target.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)
    system.automator.stop('test done')


async def test_start_on_end(system: System):
    start = system.robot_locator.pose.transform_pose(Pose(x=-1, y=0.0, yaw=0.0))
    end = system.robot_locator.pose

    def generate_path():
        return [
            PathSegment.from_poses(start, end),
        ]
    system.current_navigation = system.waypoint_navigation
    assert isinstance(system.current_navigation, WaypointNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.generate_path = generate_path  # type: ignore
    system.automator.start()
    assert system.current_navigation.target is None
    assert system.robot_locator.pose.x == pytest.approx(end.x, abs=0.1)
    assert system.robot_locator.pose.y == pytest.approx(end.y, abs=0.1)
    assert system.robot_locator.pose.yaw_deg == pytest.approx(end.yaw_deg, abs=0.1)


async def test_skip_first_segment(system: System):
    pose1 = Pose(x=-1, y=1, yaw=-np.pi/2)
    pose2 = Pose(x=0, y=0.0, yaw=0.0)
    pose3 = Pose(x=1.0, y=1.0, yaw=np.pi/2)
    pose4 = Pose(x=0, y=2.0, yaw=np.pi)

    def generate_path():
        path = [
            PathSegment.from_poses(pose1, pose2, stop_at_end=False),
            PathSegment.from_poses(pose2, pose3, stop_at_end=False),
            PathSegment.from_poses(pose3, pose4, stop_at_end=False),
            PathSegment.from_poses(pose4, pose1),
        ]
        assert isinstance(system.current_navigation, WaypointNavigation)
        path = system.current_navigation._filter_path(path)  # pylint: disable=protected-access
        return path
    system.current_navigation = system.waypoint_navigation
    assert isinstance(system.current_navigation, WaypointNavigation)
    assert isinstance(system.current_navigation.implement, Recorder)
    system.current_navigation.generate_path = generate_path  # type: ignore
    system.automator.start()
    await forward(until=lambda: system.current_navigation is not None and system.current_navigation.target is not None)
    assert system.current_navigation.target is not None
    assert len(system.current_navigation.path) == 3
    assert system.current_navigation.target.x == pytest.approx(pose3.x, abs=0.1)
    assert system.current_navigation.target.y == pytest.approx(pose3.y, abs=0.1)
    assert system.current_navigation.target.yaw_deg == pytest.approx(pose3.yaw_deg, abs=0.1)
    system.automator.stop('test done')
