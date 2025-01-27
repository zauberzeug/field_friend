from __future__ import annotations

from datetime import timedelta

import psutil
import rosys
from nicegui import ui
from rosys.analysis import track

from ..system import System
from .components import create_header
from .components.hardware_control import create_hardware_control_ui
from .components.io_overview import IoOverview as io_overview
from .components.log_monitor import LogMonitor
from .components.settings import create_settings_ui
from .components.status_dev import status_dev_page


class DevPage:

    def __init__(self, system: System) -> None:
        self.system = system
        self.log_monitor = LogMonitor()

        @ui.page('/dev')
        def page() -> None:
            create_header(system)
            track.ui()
            self.create_development_ui()

            with ui.footer():
                timer = ui.label().classes('flex-grow')
                cpu_label = ui.label().classes('flex-grow')

                async def update_status() -> None:
                    timer.set_text(f'{timedelta(seconds=rosys.uptime())}')
                    cpu_label.set_text(f'CPU: {psutil.cpu_percent():.0f}%')
                ui.timer(rosys.config.ui_update_interval, update_status)

    def create_development_ui(self) -> None:
        with ui.card().style('background-color: #2E5396; width: 100%;'):
            with ui.column().style('width: 100%;'):
                ui.label('Development Tools').style('font-size: 1.5rem; color: white;')
                create_settings_ui(self.system)
                with ui.row().style('width: 100%'):
                    with ui.card().style('background-color: #3E63A6; color: white;'):
                        if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                            with ui.row():
                                with ui.column():
                                    self.system.field_friend.robot_brain.developer_ui()
                                with ui.column():
                                    self.system.field_friend.robot_brain.communication.debug_ui()
                        else:
                            rosys.simulation_ui()
                    create_hardware_control_ui(self.system.field_friend, self.system.automator, self.system.puncher)
                    status_dev_page(self.system.field_friend, self.system)

        with ui.row():
            with ui.card():
                self.system.robot_locator.developer_ui()
            with ui.card():
                self.system.gnss.developer_ui()
            with ui.card():
                self.system.field_navigation.developer_ui()
            if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
                with ui.card().style('min-width: 200px;'):
                    esp_pins_core = self.system.field_friend.robot_brain.esp_pins_core
                    esp_pins_core.developer_ui()
                with ui.card().style('min-width: 200px;'):
                    esp_pins_p0 = self.system.field_friend.robot_brain.esp_pins_p0
                    esp_pins_p0.developer_ui()

        with ui.card().classes('w-1/2'):
            self.log_monitor.ui()
        io_overview(self.system)
