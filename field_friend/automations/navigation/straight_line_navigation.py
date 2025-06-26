from __future__ import annotations

from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.driving import PathSegment
from rosys.geometry import Point, Point3d, Pose, Spline

from ...automations.implements.implement import Implement
from .waypoint_navigation import WaypointNavigation, WorkingSegment

if TYPE_CHECKING:
    from ...system import System


class StraightLineNavigation(WaypointNavigation):
    LENGTH: float = 2.0

    def __init__(self, system: System, tool: Implement) -> None:
        super().__init__(system, tool)
        self.detector = system.detector
        self.length = self.LENGTH
        self.name = 'Straight Line'

    @property
    def target_heading(self) -> float:
        # TODO: needed?
        return self.system.robot_locator.pose.yaw

    def generate_path(self) -> list[PathSegment | WorkingSegment]:
        last_pose = self.system.robot_locator.pose
        target_pose = last_pose.transform_pose(Pose(x=self.length, y=0, yaw=last_pose.yaw))
        segment = WorkingSegment(spline=Spline.from_poses(last_pose, target_pose))
        return [segment]

    def _should_finish(self) -> bool:
        current_pose = self.system.robot_locator.pose
        if self.current_segment is None:
            return True
        return current_pose.relative_point(self.current_segment.spline.end).x <= 0

    def create_simulation(self):
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        crop_distance = 0.2
        start_point = self.robot_locator.pose.transform(Point(x=0.3, y=0))
        for i in range(0, round(self.length / crop_distance)):
            p = start_point.polar(crop_distance*i, self.robot_locator.pose.yaw) \
                .polar(randint(-2, 2)*0.01, self.robot_locator.pose.yaw+np.pi/2)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                                position=Point3d(x=p.x, y=p.y, z=0)))
            for _ in range(1, 7):
                p = start_point.polar(0.20*i+randint(-5, 5)*0.01, self.robot_locator.pose.yaw) \
                    .polar(randint(-15, 15)*0.01, self.robot_locator.pose.yaw + np.pi/2)
                self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                    position=Point3d(x=p.x, y=p.y, z=0)))

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
