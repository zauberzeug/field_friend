from __future__ import annotations

from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.analysis import track

from ...automations.implements.implement import Implement
from .navigation import Navigation

if TYPE_CHECKING:
    from ...system import System


class StraightLineNavigation(Navigation):
    LENGTH: float = 2.0

    def __init__(self, system: System, tool: Implement) -> None:
        super().__init__(system, tool)
        self.detector = system.detector
        self.length = self.LENGTH
        self.name = 'Straight Line'
        self.origin: rosys.geometry.Point
        self.target: rosys.geometry.Point

    async def prepare(self) -> bool:
        await super().prepare()
        rosys.notify(f'Activating {self.implement.name}...')
        self.update_target()
        await self.implement.activate()
        return True

    async def finish(self) -> None:
        await super().finish()
        await self.implement.deactivate()

    def update_target(self) -> None:
        self.origin = self.robot_locator.pose.point
        self.target = self.robot_locator.pose.transform(rosys.geometry.Point(x=self.length, y=0))

    @track
    async def _drive(self, distance: float) -> None:
        start_position = self.robot_locator.pose.point
        closest_point = rosys.geometry.Line.from_points(self.origin, self.target).foot_point(start_position)
        yaw = closest_point.direction(self.target)
        await self._drive_towards_target(distance, rosys.geometry.Pose(x=closest_point.x, y=closest_point.y, yaw=yaw))

    def _should_finish(self) -> bool:
        end_pose = rosys.geometry.Pose(x=self.target.x, y=self.target.y, yaw=self.origin.direction(self.target), time=0)
        return end_pose.relative_point(self.robot_locator.pose.point).x > 0

    def create_simulation(self):
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        crop_distance = 0.2
        for i in range(0, round(self.length / crop_distance)):
            p = self.robot_locator.pose.point.polar(crop_distance*i,
                                                    self.robot_locator.pose.yaw) \
                .polar(randint(-2, 2)*0.01, self.robot_locator.pose.yaw+np.pi/2)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                                position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
            for _ in range(1, 7):
                p = self.robot_locator.pose.point.polar(0.20*i+randint(-5, 5)*0.01,
                                                        self.robot_locator.pose.yaw) \
                    .polar(randint(-15, 15)*0.01, self.robot_locator.pose.yaw + np.pi/2)
                self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                    position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Length', step=0.5, min=0.05, format='%.1f', on_change=self.request_backup) \
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
