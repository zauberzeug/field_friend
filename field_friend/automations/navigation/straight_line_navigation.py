from random import randint
from typing import TYPE_CHECKING

import numpy as np
import rosys
from nicegui import ui

from field_friend.automations.implements.implement import Implement

from ..plant import Plant
from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class StraightLineNavigation(Navigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.detector = system.detector
        self.length = 2.0
        self.start_position = self.odometer.prediction.point
        self.name = 'Straight Line'

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
            await self.implement.on_focus()
        await self.implement.deactivate()

    async def _drive_forward(self):
        while not self._should_stop():
            self.log.info('driving forward...')
            target = self.odometer.prediction.transform(rosys.geometry.Point(x=0.10, y=0))
            with self.driver.parameters.set(linear_speed_limit=0.125, angular_speed_limit=0.1):
                await self.driver.drive_to(target)

    def _should_stop(self):
        distance = self.odometer.prediction.point.distance(self.start_position)
        return distance > self.length

    def create_simulation(self):
        crop_distance = 0.2
        for i in range(0, round(self.length / crop_distance)):
            p = self.odometer.prediction.point.polar(crop_distance*i,
                                                     self.odometer.prediction.yaw) \
                .polar(randint(-2, 2)*0.01, self.odometer.prediction.yaw+np.pi/2)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                                position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
            for j in range(1, 7):
                p = self.odometer.prediction.point.polar(0.20*i+randint(-5, 5)*0.01,
                                                         self.odometer.prediction.yaw) \
                    .polar(randint(-15, 15)*0.01, self.odometer.prediction.yaw + np.pi/2)
                self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='weed',
                                                                                    position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))

    def settings_ui(self) -> None:
        ui.number('Length', step=0.5, min=0.05, format='%.1f') \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'length') \
            .tooltip('Length to drive in meters')
