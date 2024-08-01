from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from .hardware_control import hardware_control
from .io_overview import io_overview
from .status_dev import status_dev_page

if TYPE_CHECKING:
    from field_friend.system import System


def development(system: 'System') -> None:
    with ui.row().style('width: calc(100vw - 2rem); flex-wrap: nowrap;'):
        with ui.card().style('background-color: #2E5396; width: 100%;'):
            with ui.column().style("width: 100%;"):
                ui.label("Development Tools").style('font-size: 1.5rem; color: white;')
                with ui.row().style("width: 100%"):
                    with ui.card().style('background-color: #3E63A6; color: white;'):
                        if isinstance(system.field_friend, rosys.hardware.RobotHardware):
                            with ui.row():
                                with ui.column():
                                    system.field_friend.robot_brain.developer_ui()
                                with ui.column():
                                    system.field_friend.robot_brain.communication.debug_ui()
                        else:
                            rosys.simulation_ui()
                    hardware_control(system.field_friend, system.automator, system.puncher)
                    status_dev_page(system.field_friend, system)
                    with ui.card().style('background-color: #3E63A6; color: white;'):
                        with ui.column():
                            ui.label("Teltonika").style('font-size: 1.5rem;')
                            ui.button("Get Status", on_click=system.teltonika_router.get_status_info)
                            ui.button("Get Sims", on_click=system.teltonika_router.get_sim_cards_info)
                            ui.button("Set Sim 1 False", on_click=lambda: system.teltonika_router.set_sim_card_activation(
                                'mob1s1a1', False))
                            ui.button("Set Sim 2 False", on_click=lambda: system.teltonika_router.set_sim_card_activation(
                                'mob1s2a1', False))
                            ui.button("Set Sim 1 True",
                                      on_click=lambda: system.teltonika_router.set_sim_card_activation('mob1s1a1', True))
                            ui.button("Set Sim 2 True",
                                      on_click=lambda: system.teltonika_router.set_sim_card_activation('mob1s2a1', True))
    with ui.row().style('width: calc(100vw - 2rem); flex-wrap: nowrap;'):
        io_overview(system)
