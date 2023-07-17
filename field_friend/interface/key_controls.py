import rosys
from nicegui.events import KeyEventArguments
from rosys import background_tasks
from rosys.automation import Automator
from rosys.driving import Steerer

from ..automations import Puncher
from ..hardware import FieldFriend


class KeyControls(rosys.driving.keyboard_control):

    def __init__(self, field_friend: FieldFriend, steerer: Steerer, automator: Automator, puncher: Puncher) -> None:
        super().__init__(steerer)
        self.field_friend = field_friend
        self.wheels = field_friend.wheels
        self.y_axis = field_friend.y_axis
        self.z_axis = field_friend.z_axis
        self.automator = automator
        self.puncher = puncher

    def handle_keys(self, e: KeyEventArguments) -> None:
        super().handle_keys(e)

        if e.modifiers.shift and e.action.keydown:
            if e.key == '!':
                self.automator.start(self.puncher.try_home())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'W':
                background_tasks.create(self.field_friend.z_axis.move(-(self.field_friend.z_axis.MAX_SPEED/4)))
            if e.key == 'S':
                background_tasks.create(self.field_friend.z_axis.move(self.field_friend.z_axis.MAX_SPEED/4))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'WS':
                background_tasks.create(self.z_axis.stop())

        if e.modifiers.shift and e.action.keydown:
            if e.key == 'A':
                background_tasks.create(self.y_axis.move_to(self.y_axis.MAX_POSITION))
            if e.key == 'D':
                background_tasks.create(self.y_axis.move_to(self.y_axis.MIN_POSITION))
        if e.modifiers.shift and e.action.keyup:
            if e.key.name in 'AD':
                background_tasks.create(self.y_axis.stop())
                rosys.notify('y axis stopped')

        if e.key == ' ' and e.action.keydown:
            background_tasks.create(self.field_friend.stop())
