from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import rosys
from rosys.geometry import Pose

from .waypoint_navigation import PathSegment, WaypointNavigation, WorkingSegment

if TYPE_CHECKING:
    from ...automations.implements.implement import Implement
    from ...system import System


class FieldNavigation(WaypointNavigation):
    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

    def generate_path(self) -> list[PathSegment | WorkingSegment]:
        field_id: str | None = self.field_provider.selected_field.id if self.field_provider.selected_field else None
        field = self.field_provider.get_field(field_id)

        if field is None:
            rosys.notify('No field selected', 'negative')
            return []
        rows_to_work_on = self.field_provider.get_rows_to_work_on()
        if not rows_to_work_on:
            rosys.notify('No rows to work on', 'negative')
            return []

        path_segments: list[PathSegment | WorkingSegment] = []
        current_pose = self.system.robot_locator.pose
        start_row_index = self._find_closest_row(rows_to_work_on)
        for i in range(len(rows_to_work_on)):
            row_idx = (start_row_index + i) % len(rows_to_work_on)
            current_row = rows_to_work_on[row_idx]
            start_point = current_row.points[0].to_local()
            end_point = current_row.points[-1].to_local()

            if current_pose.distance(end_point) < current_pose.distance(start_point):
                start_point, end_point = end_point, start_point

            start_pose = Pose(x=start_point.x, y=start_point.y, yaw=start_point.direction(end_point))
            end_pose = Pose(x=end_point.x, y=end_point.y, yaw=start_point.direction(end_point))
            if path_segments:
                path_segments.extend(self._generate_three_point_turn(current_pose, start_pose))

            path_segments.append(WorkingSegment.from_poses(start_pose, end_pose))
            current_pose = end_pose

        if path_segments:
            path_segments.insert(0, PathSegment.from_poses(self.system.robot_locator.pose, path_segments[0].start))
        return path_segments

    def _find_closest_row(self, rows: list) -> int:
        """Find the index of the closest row to the current position"""
        robot_pose = self.system.robot_locator.pose
        shortest_row_distances = [min(robot_pose.distance(row.points[0].to_local()),
                                      robot_pose.distance(row.points[-1].to_local())) for row in rows]
        return int(np.argmin(shortest_row_distances))

    def _generate_three_point_turn(self, end_pose_current_row: Pose, start_pose_next_row: Pose, radius: float = 1.5) -> list[PathSegment]:
        direction_to_start = end_pose_current_row.relative_direction(start_pose_next_row)
        distance_to_start = end_pose_current_row.distance(start_pose_next_row)
        y_offset = max(radius, distance_to_start)
        first_turn_pose = end_pose_current_row.transform_pose(
            Pose(x=radius, y=y_offset * np.sign(direction_to_start), yaw=direction_to_start))
        back_up_pose = start_pose_next_row.transform_pose(
            Pose(x=-radius, y=radius * np.sign(direction_to_start), yaw=-direction_to_start))
        return [
            PathSegment.from_poses(end_pose_current_row, first_turn_pose),
            PathSegment.from_poses(first_turn_pose, back_up_pose, backward=True),
            PathSegment.from_poses(back_up_pose, start_pose_next_row),
        ]
