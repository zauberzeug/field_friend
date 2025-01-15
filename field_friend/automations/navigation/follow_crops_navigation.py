import math
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.helpers import eliminate_2pi

from ..implements.implement import Implement
from .straight_line_navigation import StraightLineNavigation

if TYPE_CHECKING:
    from ...system import System


class FollowCropsNavigation(StraightLineNavigation):
    CROP_ATTRACTION: float = 0.3

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.plant_provider = system.plant_provider
        self.detector = system.detector
        self.start_position = self.robot_locator.pose.point
        self.flashlight = system.field_friend.flashlight
        self.plant_locator = system.plant_locator
        self.name = 'Follow Crops'
        self.crop_attraction = self.CROP_ATTRACTION

    async def prepare(self) -> bool:
        await super().prepare()
        self.log.info(f'Activating {self.implement.name}...')
        self.plant_provider.clear()
        await self.implement.activate()
        if self.flashlight:
            await self.flashlight.turn_on()
        self.plant_locator.resume()
        return True

    async def finish(self) -> None:
        await super().finish()
        if self.flashlight:
            await self.flashlight.turn_off()
        self.plant_locator.pause()
        await self.implement.deactivate()

    def update_target(self) -> None:
        self.origin = self.robot_locator.pose.point
        distance = self.length - self.robot_locator.pose.point.distance(self.start_position)
        self.target = self.robot_locator.pose.transform(rosys.geometry.Point(x=distance, y=0))

    @track
    async def _drive(self, distance: float) -> None:
        row = self.plant_provider.get_relevant_crops(point=self.robot_locator.pose.point_3d(), max_distance=1.0)
        if len(row) >= 3:
            points_array = np.array([(p.position.x, p.position.y) for p in row])
            # Fit a line using least squares
            A = np.vstack([points_array[:, 0], np.ones(len(points_array))]).T
            m, c = np.linalg.lstsq(A, points_array[:, 1], rcond=None)[0]
            yaw = np.arctan(m)
            # flip if it is pointing backwards
            if np.abs(yaw - self.robot_locator.pose.yaw) > math.pi / 2:
                yaw = yaw + math.pi
            fitted_line = rosys.geometry.Line(a=m, b=-1, c=c)
            closest_point = fitted_line.foot_point(self.robot_locator.pose.point)
            target = rosys.geometry.Pose(x=closest_point.x, y=closest_point.y, yaw=yaw)
            await self._drive_towards_target(distance, target)
        else:
            self.update_target()
            await super()._drive(distance)

    def combine_angles(self, angle1: float, influence: float, angle2: float) -> float:
        weight1 = influence
        weight2 = 1 - influence
        # Normalize both angles
        angle1 = eliminate_2pi(angle1)
        angle2 = eliminate_2pi(angle2)
        x = np.cos(angle1) * weight1 + np.cos(angle2) * weight2
        y = np.sin(angle1) * weight1 + np.sin(angle2) * weight2
        combined_angle = np.arctan2(y, x)
        # Normalize
        return eliminate_2pi(combined_angle)

    def create_simulation(self) -> None:
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        for i in range(100):
            x = i/10.0
            y = (x/4) ** 3
            if i % 10 == 0:  # create some outliers
                y += 0.2
            p = rosys.geometry.Point3d(x=x, y=y, z=0)
            p = self.robot_locator.pose.transform3d(p)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))

    def settings_ui(self) -> None:
        super().settings_ui()
        ui.number('Crop Attraction', step=0.1, min=0.0, max=1.0, format='%.1f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'crop_attraction') \
            .tooltip(f'Influence of the crop row direction on the driving direction (default: {self.CROP_ATTRACTION:.1f})')

    def backup(self) -> dict:
        return super().backup() | {
            'crop_attraction': self.crop_attraction,
        }

    def restore(self, data: dict[str, Any]) -> None:
        super().restore(data)
        self.crop_attraction = data.get('crop_attraction', self.crop_attraction)
