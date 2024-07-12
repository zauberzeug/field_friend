import asyncio
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys
from nicegui import ui

from field_friend.automations.implements.implement import Implement

from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class StraightLineNavigation(Navigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.detector = system.detector
        self.length = 2.0
        self.name = 'Straight Line'
        self.linear_speed_limit = 0.125
        self.angular_speed_limit = 0.1

    async def prepare(self) -> bool:
        await super().prepare()
        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
        return True

    async def finish(self) -> None:
        await super().finish()
        await self.implement.deactivate()

    async def _drive(self, distance: float):
        origin = self.odometer.prediction.point
        with self.driver.parameters.set(linear_speed_limit=self.linear_speed_limit, angular_speed_limit=self.angular_speed_limit):
            deadline = rosys.time() + 2
            try:
                while self.odometer.prediction.point.distance(origin) < distance:
                    if rosys.time() >= deadline:
                        raise Exception('Driving Timeout')
                    await self.driver.wheels.drive(*self.driver._throttle(1, 0.002))
                    await asyncio.sleep(0)
            finally:
                await self.driver.wheels.stop()

    def _should_finish(self):
        distance = self.odometer.prediction.point.distance(self.start_position)
        return abs(distance - self.length) < 0.05

    def create_simulation(self):
        crop_distance = 0.2
        for i in range(0, round(self.length / crop_distance)):
            p = self.odometer.prediction.point.polar(crop_distance*i,
                                                     self.odometer.prediction.yaw) \
                .polar(randint(-2, 2)*0.01, self.odometer.prediction.yaw+np.pi/2)
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(category_name='sugar_beet',
                                                                                position=rosys.geometry.Point3d(x=p.x, y=p.y, z=0)))
            for _ in range(1, 7):
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
        super().settings_ui()

    def backup(self) -> dict:
        return {
            'length': self.length,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.length = data.get('length', self.length)
