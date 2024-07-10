from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from ..components import io_sockets, status_bulb

if TYPE_CHECKING:
    from field_friend.system import System


class io_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system
        if isinstance(system.field_friend, rosys.hardware.RobotHardware):
            print(self.system.field_friend.robot_brain)
        # self.hardwaremanager = HardwareManager(self.system.field_friend.robot_brain)

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
                    if self.system.field_friend.estop.is_soft_estop_active:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Soft E-Stop')
                if self.system.is_real:
                    with ui.row():
                        ui.label('Hard E-Stop 1')
                        if self.system.field_friend.estop.pressed_estops:
                            status_bulb(True)
                        else:
                            status_bulb(False)
                    with ui.row():
                        ui.label('Hard E-Stop 2')
                        if self.system.field_friend.estop.pressed_estops:
                            status_bulb(True)
                        else:
                            status_bulb(False)
                print(self.system.field_friend.estop.pressed_estops)
                print(self.system.field_friend.estop.is_soft_estop_active)
            io_sockets(self.system)
