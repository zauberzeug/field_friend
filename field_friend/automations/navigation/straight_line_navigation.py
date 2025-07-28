from __future__ import annotations

from typing import TYPE_CHECKING, Any

from nicegui import ui
from rosys.geometry import Pose

from ...automations.implements.implement import Implement
from .waypoint_navigation import PathSegment, WaypointNavigation, WorkingSegment

if TYPE_CHECKING:
    from ...system import System


class StraightLineNavigation(WaypointNavigation):
    LENGTH: float = 2.0

    def __init__(self, system: System, tool: Implement) -> None:
        super().__init__(system, tool)
        self.length = self.LENGTH
        self.name = 'Straight Line'

    def generate_path(self) -> list[PathSegment | WorkingSegment]:
        last_pose = self.system.robot_locator.pose
        target_pose = last_pose.transform_pose(Pose(x=self.length))
        segment = WorkingSegment.from_poses(last_pose, target_pose)
        return [segment]

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Length', step=0.5, min=0.05, format='%.1f', suffix='m', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'length') \
            .tooltip('Length to drive in meters')

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
            'length': self.length,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        super().restore_from_dict(data)
        self.length = data.get('length', self.length)
