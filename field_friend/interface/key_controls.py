from typing import TYPE_CHECKING

import rosys
from nicegui.events import KeyEventArguments
from rosys import background_tasks

from ..hardware import YAxis

if TYPE_CHECKING:
    from field_friend.system import System


class KeyControls(rosys.driving.keyboard_control):

    def __init__(self, system: 'System') -> None:
        super().__init__(system.steerer)
        self.field_friend = system.field_friend
        self.wheels = system.field_friend.wheels
        self.y_axis = system.field_friend.y_axis
        self.z_axis = system.field_friend.z_axis
        self.automator = system.automator
        self.puncher = system.puncher
        self.system = system
        self.estop_on_space = True

    def handle_keys(self, e: KeyEventArguments) -> None:
        super().handle_keys(e)

        if e.modifiers.shift and e.action.keydown:
            if e.key == '!':
                self.automator.start(self.puncher.try_home())
        if self.z_axis is not None:
            if e.modifiers.shift and e.action.keydown:
                if e.key == 'W':
                    background_tasks.create(self.field_friend.z_axis.move(-(self.field_friend.z_axis.MAX_SPEED/6)))
                if e.key == 'S':
                    background_tasks.create(self.field_friend.z_axis.move(self.field_friend.z_axis.MAX_SPEED/6))
            if e.modifiers.shift and e.action.keyup:
                if e.key.name in 'WS':
                    background_tasks.create(self.z_axis.stop())

        if isinstance(self.y_axis, YAxis):
            if e.modifiers.shift and e.action.keydown:
                if e.key == 'A':
                    background_tasks.create(self.y_axis.move_to(self.y_axis.MAX_POSITION))
                if e.key == 'D':
                    background_tasks.create(self.y_axis.move_to(self.y_axis.MIN_POSITION))
            if e.modifiers.shift and e.action.keyup:
                if e.key.name in 'AD':
                    background_tasks.create(self.y_axis.stop())
                    rosys.notify('y axis stopped')

        if e.action.keydown and e.key == 'r' and e.modifiers.shift:
            self.system.restart()

        if e.action.keydown and e.key == 's':
            if self.automator.is_running:
                self.automator.stop(because='stop button was pressed')
            else:
                self.automator.start()

        # if e.key == ' ' and e.action.keydown:
        #     background_tasks.create(self.field_friend.stop())
        #     if not self.estop_on_space:
        #         return
        #     if not self.field_friend.estop.is_soft_estop_active:
        #         background_tasks.create(self.field_friend.estop.set_soft_estop(True))
        #         rosys.notify('ESTOP activated')
        #     else:
        #         background_tasks.create(self.field_friend.estop.set_soft_estop(False))
        #         rosys.notify('ESTOP released')
