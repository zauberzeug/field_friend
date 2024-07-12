from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from ..components import io_sockets, status_bulb

if TYPE_CHECKING:
    from field_friend.system import System


class io_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/io')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        with ui.card().style('background-color: #2E5396; color: white;'), ui.row():
            with ui.row().style("width: 100%"):
                ui.label("I/O Overview").style('font-size: 1.5rem; color: white;')
            with ui.card().style('background-color: #3E63A6;') as estop_card:
                ui.markdown('**E-Stops**').classes('w-full text-center')
                ui.separator()
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.estop, 'is_soft_estop_active')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.estop, 'is_soft_estop_active', lambda x: not x)
                    ui.label('Soft E-Stop')
                if self.system.is_real:
                    with ui.row():
                        # TODO: Need to test this, because not sure if '1' and '2' are correct)
                        status_bulb(True).bind_visibility_from(self.system.field_friend.estop,
                                                               'pressed_estops', lambda pressed_estops: '1' in pressed_estops)
                        status_bulb(False).bind_visibility_from(
                            self.system.field_friend.estop, 'pressed_estops', lambda pressed_estops: '1' not in pressed_estops or not pressed_estops)
                        ui.label('Hard E-Stop 1')
                    with ui.row():
                        status_bulb(True).bind_visibility_from(self.system.field_friend.estop,
                                                               'pressed_estops', lambda pressed_estops: '2' in pressed_estops)
                        status_bulb(False).bind_visibility_from(
                            self.system.field_friend.estop, 'pressed_estops', lambda pressed_estops: '2' not in pressed_estops or not pressed_estops)
                        ui.label('Hard E-Stop 2')
            io_sockets(self.system)
