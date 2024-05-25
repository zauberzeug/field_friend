import rosys
from rosys.driving.driver import Driver

from field_friend.automations.tool.tool import Tool

from .navigation import Navigation


class StraightLineNavigation(Navigation):

    def __init__(self,
                 driver: rosys.driving.Driver,
                 odometer: rosys.driving.Odometer,
                 tool: Tool,
                 ) -> None:
        super().__init__(driver, odometer, tool)
        self.length = 2.0
        self.start_position = self.odometer.prediction.point

    async def start(self):
        self.start_position = self.odometer.prediction.point
        if not await self.tool.prepare():
            self.log.error('Tool-Preparation failed')
            return
        self.log.info('driving straight line forward...')
        await self.tool.activate()
        while True:
            await rosys.automation.parallelize(
                self.tool.observe(),
                self._drive_forward(),
                return_when_first_completed=True
            )
            await self.tool.on_focus()
            if await self._should_stop():
                break
        await self.tool.deactivate()

    async def _drive_forward(self):
        while not await self._should_stop():
            self.log.info('driving forward...')
            target = self.odometer.prediction.transform(rosys.geometry.Point(x=0.10, y=0))
            await self.driver.drive_to(target)

    async def _should_stop(self):
        distance = self.odometer.prediction.point.distance(self.start_position)
        return distance > self.length
