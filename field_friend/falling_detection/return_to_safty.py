from rosys.driving import Steerer
from nicegui import ui
from .check_falling import Falling
class RuturnToSafety:

    def __init__(self,steerer: Steerer,falling_detection: Falling) -> None:
        self.steerer = steerer
        self.rescueing = False
        self.falling_detection = falling_detection

    async def rescue(self):
        await self.falling_detection.field_friend.estop.set_soft_estop(False)
        self.rescueing = True
        self.falling_detection.rescue_in_progress()
        self.steerer.start()
        self.steerer.update(0,-0.5)
        ui.timer(3,self.wait_function,once=True)
    
    def wait_function(self):
        self.steerer.stop()
        self.rescueing = False
        self.falling_detection.reset()