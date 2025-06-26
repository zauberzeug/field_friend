from __future__ import annotations

from typing import TYPE_CHECKING

import rosys
from rosys.driving import PathSegment
from rosys.geometry import Point, Pose, Spline

from .navigation import Navigation

if TYPE_CHECKING:
    from ...automations.implements.implement import Implement
    from ...system import System


WAYPOINTS = [Point(x=3.0 * x, y=x % 2) for x in range(1, 5)]


class WaypointNavigation(Navigation):
    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Waypoint Navigation'
        self._upcoming_path: list[PathSegment | WorkingSegment] = []

    @property
    def path(self) -> list[PathSegment | WorkingSegment]:
        return self._upcoming_path

    @property
    def current_segment(self) -> PathSegment | WorkingSegment | None:
        if not self._upcoming_path:
            return None
        return self._upcoming_path[0]

    def generate_path(self) -> list[PathSegment | WorkingSegment]:
        last_pose = self.system.robot_locator.pose
        path: list[PathSegment | WorkingSegment] = []
        segment: PathSegment | WorkingSegment
        for waypoint in WAYPOINTS:
            next_pose = Pose(x=waypoint.x, y=waypoint.y, yaw=last_pose.yaw)
            if next_pose.y % 2:
                segment = WorkingSegment(spline=Spline.from_poses(last_pose, next_pose))
            else:
                segment = PathSegment(spline=Spline.from_poses(last_pose, next_pose))
            path.append(segment)
            last_pose = next_pose
        return path

    @property
    def target_heading(self) -> float:
        # TODO: needed?
        return 0

    async def prepare(self) -> bool:
        await super().prepare()
        self._upcoming_path = self.generate_path()
        if self._should_finish():
            return False
        return True

    async def _drive(self) -> None:
        while not self._should_finish():
            segment = self.current_segment
            if segment is None:
                return
            stop_at_end = len(self._upcoming_path) == 1
            stop_at_end = True
            if isinstance(segment, WorkingSegment):
                await self.driver.drive_spline(segment.spline, throttle_at_end=stop_at_end, stop_at_end=stop_at_end)
            else:
                with self.implement.blocked():
                    await self.driver.drive_spline(segment.spline, throttle_at_end=stop_at_end, stop_at_end=stop_at_end)
            await rosys.sleep(0.5)
            self._upcoming_path.pop(0)

    def _should_finish(self) -> bool:
        return self.current_segment is None

    def developer_ui(self) -> None:
        pass


class WorkingSegment(PathSegment):
    ...
