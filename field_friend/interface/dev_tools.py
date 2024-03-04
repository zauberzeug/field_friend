from nicegui import ui

from field_friend.system import System

from .development import development
from .hardware_control import hardware_control
from .status_dev_page import status_dev_page


def dev_tools(system: System) -> None:
    with ui.card().style('background-color: #2E5396; width: 100%;'):
        with ui.row().style('width: 100%;'):
            with ui.column().style("width: calc(100% - 350px); max-width: calc(100% - 350px);"):
                ui.label("Development Tools").style('font-size: 1.5rem; color: white;')
                with ui.row().style("width: 100%"):
                    development(system.field_friend)
                    hardware_control(system.field_friend, system.automator, system.puncher)
            with ui.card().style('width: 300px; height: 70vh; background-color: #2E5396;'):
                status_dev_page(system.field_friend, system.gnss, system.odometer)
