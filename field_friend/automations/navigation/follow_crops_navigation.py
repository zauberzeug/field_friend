import numpy as np
import rosys
from rosys.helpers import eliminate_2pi

from ..implements.implement import Implement
from ..kpi_provider import KpiProvider
from ..plant_provider import PlantProvider
from .navigation import Navigation


class FollowCropsNavigation(Navigation):
    DRIVE_DISTANCE = 0.04

    def __init__(self,
                 driver: rosys.driving.Driver,
                 odometer: rosys.driving.Odometer,
                 kpi_provider: KpiProvider,
                 tool: Implement,
                 plant_provider: PlantProvider,
                 ) -> None:
        super().__init__(driver, odometer, kpi_provider, tool)
        self.plant_provider = plant_provider
        self.start_position = self.odometer.prediction.point
        self.name = 'Follow Crops'
        self._should_stop = False

    async def _start(self):
        self.start_position = self.odometer.prediction.point
        if not await self.implement.prepare():
            self.log.error('Tool-Preparation failed')
            return
        self.log.info('following crops ...')
        await self.implement.activate()
        while True:
            await rosys.automation.parallelize(
                self.implement.observe(),
                self._drive_forward(),
                return_when_first_completed=True
            )
            await self.implement.on_focus()
            if self._should_stop:
                break
        await self.implement.deactivate()

    async def _drive_forward(self):
        while True:
            row = self.plant_provider.get_relevant_crops(self.odometer.prediction.point)
            if not row:
                # TODO: make stop condition more sophisticated and configurable (like "stop after 40 cm without crops")
                self._should_stop = True
                break
            if len(row) >= 2:
                points_array = np.array([(p.position.x, p.position.y) for p in row])
                # Fit a line using least squares
                A = np.vstack([points_array[:, 0], np.ones(len(points_array))]).T
                m, c = np.linalg.lstsq(A, points_array[:, 1], rcond=None)[0]
                yaw_of_row = np.arctan(m)
            else:
                yaw_of_row = self.odometer.prediction.yaw
            # only apply some yaw corrections to avoid oversteering
            target_yaw = self._weighted_angle_combine(self.odometer.prediction.yaw, 0.65, yaw_of_row, 0.35)
            target = self.odometer.prediction.point.polar(self.DRIVE_DISTANCE, target_yaw)
            self.log.info(f'Current world position: {self.odometer.prediction} Target next crop at {target}')
            with self.driver.parameters.set(linear_speed_limit=0.125, angular_speed_limit=0.1):
                await self.driver.drive_to(target)

    def _weighted_angle_combine(self, angle1: float, weight1: float, angle2: float, weight2: float) -> float:
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

    # def create_simulation(self):
    #     for i in range(100):
    #         x = i/10.0
    #         p = rosys.geometry.Point3d(x=x, y=np.sin(x/2), z=0)
    #         self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='maize', position=p))
