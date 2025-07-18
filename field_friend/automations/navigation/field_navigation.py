from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Self

import numpy as np
import rosys
from rosys import helpers
from rosys.geometry import Pose

from ..field import Row
from .waypoint_navigation import PathSegment, WaypointNavigation, WorkingSegment

if TYPE_CHECKING:
    from ...automations.implements.implement import Implement
    from ...system import System


class FieldNavigation(WaypointNavigation):
    MAX_START_DISTANCE = 2.0
    MAX_DISTANCE_DEVIATION = 0.1
    MAX_ANGLE_DEVIATION = np.deg2rad(15.0)

    def __init__(self, system: System, implement: Implement) -> None:
        super().__init__(system, implement)
        self.name = 'Field Navigation'
        self.gnss = system.gnss
        self.bms = system.field_friend.bms
        self.automator = system.automator
        self.automation_watcher = system.automation_watcher
        self.field_provider = system.field_provider

    async def prepare(self) -> bool:
        await super().prepare()
        if not self._is_allowed_to_start():
            return False
        return True

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
        row_reversed = self._is_row_reversed(rows_to_work_on[start_row_index])
        for i, row in enumerate(rows_to_work_on):
            if i < start_row_index:
                continue
            row_segment = RowSegment.from_row(row, reverse=row_reversed)
            if path_segments:
                path_segments.extend(self._generate_three_point_turn(current_pose, row_segment.start))
            path_segments.append(row_segment)
            current_pose = row_segment.end
            row_reversed = not row_reversed

        current_pose = self.system.robot_locator.pose
        t = path_segments[0].spline.closest_point(current_pose.x, current_pose.y, t_min=-0.1, t_max=1.1)
        if t < 0:
            path_segments.insert(0, PathSegment.from_poses(self.system.robot_locator.pose, path_segments[0].start))
        return path_segments

    def _find_closest_row(self, rows: list) -> int:
        """Find the index of the closest row to the current position"""
        robot_pose = self.system.robot_locator.pose
        shortest_row_distances = [min(robot_pose.distance(row.points[0].to_local()),
                                      robot_pose.distance(row.points[-1].to_local())) for row in rows]
        return int(np.argmin(shortest_row_distances))

    def _is_row_reversed(self, row: Row) -> bool:
        current_pose = self.system.robot_locator.pose
        row_start = row.points[0].to_local()
        row_end = row.points[-1].to_local()
        relative_start = current_pose.relative_point(row_start)
        relative_end = current_pose.relative_point(row_end)
        robot_in_working_area = relative_start.x * relative_end.x <= 0.0
        if robot_in_working_area:
            return relative_start.x > 0
        distance_to_start = current_pose.distance(row_start)
        distance_to_end = current_pose.distance(row_end)
        return distance_to_start > distance_to_end

    def _is_allowed_to_start(self) -> bool:
        first_row_segment = next((segment for segment in self._upcoming_path if isinstance(segment, RowSegment)), None)
        if first_row_segment is None:
            return False
        current_pose = self.system.robot_locator.pose
        relative_start = current_pose.relative_point(first_row_segment.start.point)
        relative_end = current_pose.relative_point(first_row_segment.end.point)
        robot_in_working_area = relative_start.x * relative_end.x <= 0.0
        if robot_in_working_area:
            t = first_row_segment.spline.closest_point(current_pose.x, current_pose.y)
            spline_pose = first_row_segment.spline.pose(t)
            if current_pose.distance(spline_pose) > self.MAX_DISTANCE_DEVIATION:
                rosys.notify('Distance to row is too large', 'negative')
                return False
            if abs(helpers.angle(current_pose.yaw, spline_pose.yaw)) > self.MAX_ANGLE_DEVIATION:
                rosys.notify('Robot is not aligned with the row', 'negative')
                return False
        else:
            if abs(current_pose.relative_direction(first_row_segment.start)) > self.MAX_ANGLE_DEVIATION:
                rosys.notify('Robot is not aligned with the row', 'negative')
                return False
            if abs(relative_start.x) > self.MAX_START_DISTANCE:
                rosys.notify('Robot is too far from the row', 'negative')
                return False
        return True

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


@dataclass(slots=True, kw_only=True)
class RowSegment(WorkingSegment):
    row: Row

    @classmethod
    def from_row(cls, row: Row, *, reverse: bool = False) -> Self:
        start_point = row.points[0].to_local()
        end_point = row.points[-1].to_local()
        if reverse:
            start_point, end_point = end_point, start_point
        start_pose = Pose(x=start_point.x, y=start_point.y, yaw=start_point.direction(end_point))
        end_pose = Pose(x=end_point.x, y=end_point.y, yaw=start_point.direction(end_point))
        segment = WorkingSegment.from_poses(start_pose, end_pose)
        return cls(row=row, spline=segment.spline, backward=segment.backward, stop_at_end=segment.stop_at_end)
