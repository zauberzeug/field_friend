from __future__ import annotations

from datetime import timedelta

import numpy as np
import psutil
import rosys
from nicegui import ui
from rosys.analysis import track
from rosys.helpers import eliminate_2pi

from ..hardware.sprayer import Sprayer
from ..system import System
from ..vision.detector_hardware import DetectorHardware
from .components import create_header
from .components.hardware_control import create_hardware_control_ui
from .components.io_overview import IoOverview as io_overview
from .components.log_monitor import LogMonitor
from .components.settings import create_settings_ui
from .components.status_bulb import StatusBulb as status_bulb
from .components.status_dev import status_dev_page


class DevPage:
    """The development page gives are more detailed overview of the robot's status, that is more helpful for developers than for users.

    This page provides detailed information about the robot's hardware components,
    including the robot brain, communication, and other peripherals. It also includes
    controls for the robot's hardware components, such as the wheels, IMU, and GNSS.
    """

    def __init__(self, system: System, log_monitor: LogMonitor) -> None:
        self.system = system
        self.log_monitor = log_monitor

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
                    if isinstance(self.system.field_friend.z_axis, Sprayer):
                        with ui.card():
                            with ui.column():
                                self.system.field_friend.z_axis.developer_ui()

        with ui.row():
            with ui.card():
                self.system.robot_locator.developer_ui()
            with ui.card():
                with ui.row():
                    with ui.column():
                        if isinstance(self.system.field_friend.wheels, rosys.hardware.WheelsSimulation):
                            self.wheels_ui()
                    with ui.column():
                        self.odometer_ui()
            if self.system.gnss is not None:
                with ui.card():
                    self.system.gnss.developer_ui()
                    ui.button('Update reference', on_click=self.system.update_gnss_reference)
            if isinstance(self.system.field_friend.imu, rosys.hardware.Imu):
                with ui.card():
                    self.system.field_friend.imu.developer_ui()

        with ui.row():
            if self.system.field_navigation is not None:
                with ui.card():
                    self.system.field_navigation.developer_ui()
            if self.system.plant_locator is not None:
                with ui.card():
                    self.system.plant_locator.developer_ui()

        with ui.row():
            with ui.card():
                self.system.field_friend.bms.developer_ui()
                if hasattr(self.system.field_friend, 'battery_control') and self.system.field_friend.battery_control is not None:
                    with ui.row():
                        ui.label('Out 1..4 Status:').tooltip('Battery Box out connectors 1-4')
                        status_bulb().bind_value_from(self.system.field_friend.battery_control, 'status')

        if isinstance(self.system.field_friend, rosys.hardware.RobotHardware):
            with ui.row():
                with ui.card().style('min-width: 200px;'):
                    esp_pins_core = self.system.field_friend.robot_brain.esp_pins_core
                    esp_pins_core.developer_ui()
                with ui.card().style('min-width: 200px;'):
                    esp_pins_p0 = self.system.field_friend.robot_brain.esp_pins_p0
                    esp_pins_p0.developer_ui()

        if self.system.plant_locator is not None:
            with ui.row():
                with ui.card():
                    self.system.plant_locator.developer_ui()
                if isinstance(self.system.detector, DetectorHardware):
                    with ui.card():
                        self.system.detector.developer_ui()

        with ui.card().classes('w-1/2'):
            self.log_monitor.ui()
        io_overview(self.system)

    def odometer_ui(self) -> None:
        ui.label('Odometer').classes('text-center text-bold')
        ui.label().bind_text_from(self.system.odometer, 'prediction',
                                  backward=lambda prediction: f'x: {prediction.x:.3f} m')
        ui.label().bind_text_from(self.system.odometer, 'prediction',
                                  backward=lambda prediction: f'y: {prediction.y:.3f} m')
        ui.label().bind_text_from(self.system.odometer, 'prediction',
                                  backward=lambda prediction: f'yaw: {prediction.yaw_deg:.2f} °')

    def wheels_ui(self) -> None:
        assert isinstance(self.system.field_friend.wheels, rosys.hardware.WheelsSimulation)
        with ui.column():
            ui.label('Wheels').classes('text-center text-bold')
            ui.label().bind_text_from(self.system.field_friend.wheels, 'pose',
                                      backward=lambda pose: f'x: {pose.x:.3f} m')
            ui.label().bind_text_from(self.system.field_friend.wheels, 'pose',
                                      backward=lambda pose: f'y: {pose.y:.3f} m')
            ui.label().bind_text_from(self.system.field_friend.wheels, 'pose',
                                      backward=lambda pose: f'yaw: {np.rad2deg(eliminate_2pi(pose.yaw)):.2f} °')
            ui.number('slip_factor_right', min=-1, max=1, step=0.01, value=0, format='%.2f') \
                .bind_value(self.system.field_friend.wheels, 'slip_factor_right') \
                .classes('w-24')
            ui.number('slip_factor_left', min=-1, max=1, step=0.01, value=0, format='%.2f') \
                .bind_value(self.system.field_friend.wheels, 'slip_factor_left') \
                .classes('w-24')
