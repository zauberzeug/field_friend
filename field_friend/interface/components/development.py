from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.hardware import EspPins

from .hardware_control import create_hardware_control_ui
from .io_overview import IoOverview as io_overview
from .settings import create_settings_ui
from .status_dev import status_dev_page

if TYPE_CHECKING:
    from ...system import System


def create_development_ui(system: 'System') -> None:
    with ui.row().style('width: calc(100vw - 2rem); flex-wrap: nowrap;'):
        with ui.card().style('background-color: #2E5396; width: 100%;'):
            with ui.column().style('width: 100%;'):
                ui.label('Development Tools').style('font-size: 1.5rem; color: white;')
                create_settings_ui(system)
                with ui.row().style('width: 100%'):
                    with ui.card().style('background-color: #3E63A6; color: white;'):
                        if isinstance(system.field_friend, rosys.hardware.RobotHardware):
                            with ui.row():
                                with ui.column():
                                    system.field_friend.robot_brain.developer_ui()
                                with ui.column():
                                    system.field_friend.robot_brain.communication.debug_ui()
                        else:
                            rosys.simulation_ui()
                    create_hardware_control_ui(system.field_friend, system.automator, system.puncher)
                    status_dev_page(system.field_friend, system)
                    with ui.card().style('background-color: #3E63A6; color: white;'):
                        ui.label('Field Navigation')
                        system.field_navigation.developer_ui()
    with ui.row().style('width: calc(100vw - 2rem); flex-wrap: nowrap;'):
        io_overview(system)
    if isinstance(system.field_friend, rosys.hardware.RobotHardware):
        with ui.row():
            with ui.card().style('min-width: 200px;'):
                esp_pins_core = EspPins(name='core', robot_brain=system.field_friend.robot_brain)
                esp_pins_core.developer_ui()
            with ui.card().style('min-width: 200px;'):
                esp_pins_p0 = EspPins(name='p0', robot_brain=system.field_friend.robot_brain)
                esp_pins_p0.developer_ui()
