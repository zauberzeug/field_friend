from typing import TYPE_CHECKING

import rosys

from field_friend.automations.implements.implement import Implement

from .navigation import Navigation

if TYPE_CHECKING:
    from system import System


class StraightLineNavigation(Navigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
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
