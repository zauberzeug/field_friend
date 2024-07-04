import math
from typing import TYPE_CHECKING, Any

import numpy as np

import rosys
from nicegui import ui
from rosys.helpers import eliminate_2pi

from ..implements.implement import Implement
from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class FollowCropsNavigation(Navigation):
    DRIVE_DISTANCE = 0.02

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.plant_provider = system.plant_provider
        self.detector = system.detector
        self.start_position = self.odometer.prediction.point
        self.flashlight = system.field_friend.flashlight
        self.plant_locator = system.plant_locator
        self.name = 'Follow Crops'
        self.crop_attraction = 0.5

    async def prepare(self) -> bool:
        self.log.info(f'Activating {self.implement.name}...')
        self.plant_provider.clear()
        await self.implement.activate()
        await self.flashlight.turn_on()
        self.plant_locator.resume()
        return True

    async def finish(self) -> None:
        await self.flashlight.turn_off()
        self.plant_locator.pause()
        await self.implement.deactivate()

    async def _drive(self):
        row = self.plant_provider.get_relevant_crops(self.odometer.prediction.point, max_distance=1.0)
        if len(row) >= 3:
            points_array = np.array([(p.position.x, p.position.y) for p in row])
            # Fit a line using least squares
            A = np.vstack([points_array[:, 0], np.ones(len(points_array))]).T
            m, c = np.linalg.lstsq(A, points_array[:, 1], rcond=None)[0]
            yaw = np.arctan(m)
            # flip if it is pointing backwards
            if np.abs(yaw - self.odometer.prediction.yaw) > math.pi / 2:
                yaw = yaw + math.pi

            # Calculate a point 0.3 meters in front of the robot along the line
            x_front = self.odometer.prediction.point.x + 0.3 * np.cos(yaw)
            y_front = m * x_front + c
            point_front = np.array([x_front, y_front])

            # Calculate the desired yaw angle from the robot's current position to the front point
            delta_x = point_front[0] - self.odometer.prediction.point.x
            delta_y = point_front[1] - self.odometer.prediction.point.y
            yaw_of_row = np.arctan2(delta_y, delta_x)
        else:
            yaw_of_row = self.odometer.prediction.yaw
        target_yaw = self.combine_angles(yaw_of_row, self.crop_attraction, self.odometer.prediction.yaw)
        # self.log.info(f'following crops with target yaw {target_yaw}')
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
        x = np.cos(angle1) * weight1 + np.cos(angle2) * weight2
        y = np.sin(angle1) * weight1 + np.sin(angle2) * weight2
        combined_angle = np.arctan2(y, x)
        # Normalize
        return eliminate_2pi(combined_angle)

    def create_simulation(self):
        for i in range(100):
            x = i/10.0
            p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))

    def _should_finish(self) -> bool:
        distance = self.odometer.prediction.point.distance(self.start_position)
        if distance < 0.5:
            return False  # at least drive 0.5m
        if len(self.plant_provider.get_relevant_crops(self.odometer.prediction.point)) == 0:
            return True
        return False

    def settings_ui(self) -> None:
        ui.number('Crop Attraction', step=0.1, min=0.0, format='%.1f') \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'crop_attraction') \
            .tooltip('Influence of the crop row direction on the driving direction')
        super().settings_ui()

    def backup(self) -> dict:
        return {
            'crop_attraction': self.crop_attraction,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.crop_attraction = data.get('crop_attraction', self.crop_attraction)
