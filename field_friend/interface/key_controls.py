import rosys
from nicegui.events import KeyEventArguments
from rosys import background_tasks
from rosys.automation import Automator
from rosys.driving import Steerer
from rosys.hardware import Wheels

from ..hardware import EStop, YAxis, ZAxis
from ..old_hardware import Robot


class KeyControls(rosys.driving.keyboard_control):

    def __init__(self, wheels: Wheels, y_axis: YAxis, z_axis: ZAxis, steerer: Steerer, automator: Automator) -> None:
        super().__init__(steerer)

        self.wheels = wheels
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.automator = automator

    def handle_keys(self, e: KeyEventArguments) -> None:
        super().handle_keys(e)

        # if e.modifiers.shift and e.action.keydown:
        #     if e.key == '!':
        #         async def try_axis_home():
        #             if not await self.robot.start_homing():
        #                 rosys.notify('homing: failed')
        #             else:
        #                 rosys.notify('homing successful')
        #         self.automator.start(try_axis_home())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'W':
                background_tasks.create(self.z_axis.move_to(self.z_axis.MAX_Z))
            if e.key == 'S':
                background_tasks.create(self.z_axis.move_to(self.z_axis.MIN_Z))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'WS':
                background_tasks.create(self.z_axis.stop())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'A':
                background_tasks.create(self.y_axis.move_to(self.y_axis.MAX_Y))
            if e.key == 'D':
                background_tasks.create(self.y_axis.move_to(self.y_axis.MIN_Y))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'AD':
                background_tasks.create(self.y_axis.stop())

        if e.key == ' ' and e.action.keydown:
            background_tasks.create(self.wheels.stop())
            # TODO: stopp all background tasks when the operation is stopped
