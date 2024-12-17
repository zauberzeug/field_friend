from __future__ import annotations

import math
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


def create_development_ui(system: System) -> None:
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

    with ui.row():
        with ui.card():
            system.robot_locator.developer_ui()
        with ui.card():
            system.gnss.developer_ui()
        if isinstance(system.field_friend.wheels, rosys.hardware.WheelsSimulation):
            with ui.card():
                ui.label().bind_text_from(system.field_friend.wheels.pose, 'x', lambda x: f'x: {x:.3f} m')
                ui.label().bind_text_from(system.field_friend.wheels.pose, 'y', lambda y: f'y: {y:.3f} m')
                ui.label().bind_text_from(system.field_friend.wheels.pose, 'yaw_deg', lambda yaw: f'yaw: {yaw:.2f} °')
                ui.label().bind_text_from(system.field_friend.wheels, 'linear_velocity', lambda v: f'v: {v:.3f} m/s')
                ui.label().bind_text_from(system.field_friend.wheels, 'angular_velocity',
                                          lambda omega: f'ω: {math.degrees(omega):.3f} °/s')

        with ui.card():
            ui.label('Odometry').classes('text-center text-bold')
            ui.label().bind_text_from(system.odometer, 'prediction', lambda prediction: f'x: {prediction.x:.3f}')
            ui.label().bind_text_from(system.odometer, 'prediction', lambda prediction: f'y: {prediction.y:.3f}')
            ui.label().bind_text_from(system.odometer, 'prediction',
                                      lambda prediction: f'yaw: {prediction.yaw_deg:.2f}')
        with ui.card():
            async def turn_to_yaw(yaw: float) -> None:
                rosys.notify(f'turning to {yaw}°', type='info')
                await system.field_navigation.turn_to_yaw(math.radians(yaw))
                rosys.notify(f'turned to {yaw}°', type='positive')

            ui.button('0°', on_click=lambda: turn_to_yaw(0.0))
            ui.button('90°', on_click=lambda: turn_to_yaw(90.0))
            ui.button('180°', on_click=lambda: turn_to_yaw(180.0))
            ui.button('270°', on_click=lambda: turn_to_yaw(270.0))
            ui.label(f'{rosys.time()}')

        with ui.card():
            system.field_navigation.developer_ui()
        if isinstance(system.field_friend, rosys.hardware.RobotHardware):
            with ui.card().style('min-width: 200px;'):
                esp_pins_core = EspPins(name='core', robot_brain=system.field_friend.robot_brain)
                esp_pins_core.developer_ui()
            with ui.card().style('min-width: 200px;'):
                esp_pins_p0 = EspPins(name='p0', robot_brain=system.field_friend.robot_brain)
                esp_pins_p0.developer_ui()

    io_overview(system)
