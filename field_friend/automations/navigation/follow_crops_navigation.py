from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui
from rosys.helpers import eliminate_2pi

from ..implements.implement import Implement
from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class FollowCropsNavigation(Navigation):
    DRIVE_DISTANCE = 0.04

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.plant_provider = system.plant_provider
        self.detector = system.detector
        self.start_position = self.odometer.prediction.point
        self.name = 'Follow Crops'

    async def _start(self):
        self.start_position = self.odometer.prediction.point
        if not await self.implement.prepare():
            self.log.error('Tool-Preparation failed')
            return
        while not self._should_stop():
            await rosys.automation.parallelize(
                self.implement.observe(),
                self._drive_forward(),
                return_when_first_completed=True
            )
            await self.implement.start_workflow()
            await self.implement.stop_workflow()
        await self.implement.deactivate()

    async def _drive_forward(self):
        while not self._should_stop():
            row = self.plant_provider.get_relevant_crops(self.odometer.prediction.point)
            if len(row) >= 2:
                points_array = np.array([(p.position.x, p.position.y) for p in row])
                # Fit a line using least squares
                A = np.vstack([points_array[:, 0], np.ones(len(points_array))]).T
                m, c = np.linalg.lstsq(A, points_array[:, 1], rcond=None)[0]
                yaw_of_row = np.arctan(m)
            else:
                yaw_of_row = self.odometer.prediction.yaw
            # NOTE we only apply some yaw corrections to avoid oversteering
            target_yaw = self.combine_angles(yaw_of_row, 0.6, self.odometer.prediction.yaw)
            target = self.odometer.prediction.point.polar(self.DRIVE_DISTANCE, target_yaw)
            # self.log.info(f'Current world position: {self.odometer.prediction} Target next crop at {target}')
            with self.driver.parameters.set(linear_speed_limit=0.125, angular_speed_limit=0.1):
                await self.driver.drive_to(target)

    def combine_angles(self, angle1: float, influence: float, angle2: float) -> float:
        weight1 = influence
        weight2 = 1 - influence
        # Normalize both angles
        angle1 = eliminate_2pi(angle1)
        angle2 = eliminate_2pi(angle2)
        # Combine angles with the weights
        x = np.cos(angle1) * weight1 + np.cos(angle2) * weight2
        y = np.sin(angle1) * weight1 + np.sin(angle2) * weight2
        # Compute the resultant angle
        combined_angle = np.arctan2(y, x)
        # Normalize the resultant angle
        return eliminate_2pi(combined_angle)

    def create_simulation(self):
        for i in range(100):
            x = i/10.0
            p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))

    def _should_stop(self):
        distance = self.odometer.prediction.point.distance(self.start_position)
        if distance < 0.5:
            return False  # at least drive 0.5m
        if len(self.plant_provider.get_relevant_crops(self.odometer.prediction.point)) == 0:
            return True

    def settings_ui(self) -> None:
        ui.label('no settings yet')
