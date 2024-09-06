from typing import TYPE_CHECKING, Optional

import rosys
from nicegui import events, ui

from .key_controls import KeyControls

if TYPE_CHECKING:
    from field_friend.system import System

SHORTCUT_INFO = '''
    Steer the robot manually with the JOYSTICK. <br>
    Or hold SHIFT and use the ARROW KEYS
'''


class manual_steerer_dialog(ui.dialog):
    def __init__(self, system: 'System') -> None:
        super().__init__()
        self.system = system
        with self, ui.card().classes('place-items-center'):
            self.key_controls = KeyControls(self.system)
            rosys.driving.joystick(self.system.steerer, size=50, color='#6E93D6')
            with ui.column():
                ui.markdown(SHORTCUT_INFO).classes('col-grow')
            with ui.row().classes('w-full justify-end'):
                ui.number('speed', format='%.0f', max=4, min=1, value=2).props('dense outlined').classes(
                    'w-24 mr-4').bind_value(self.key_controls, 'speed').tooltip('Set the speed of the robot (1-4)')
                ui.space()
                ui.button('Close', on_click=self.close).props('outline')
