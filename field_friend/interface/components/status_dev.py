from __future__ import annotations

from datetime import timedelta
from typing import TYPE_CHECKING

import psutil
import rosys
from nicegui import ui

from ...hardware import (
    Axis,
    ChainAxis,
    DoubleWheelsHardware,
    FieldFriend,
    FieldFriendHardware,
    FlashlightPWMHardware,
    FlashlightPWMHardwareV2,
    Tornado,
    TornadoHardware,
)

if TYPE_CHECKING:
    from ...system import System


def status_dev_page(robot: FieldFriend, system: System):
    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.label('Hardware').style('color: #6E93D6;').classes('w-full text-center font-bold')
        ui.separator()

        with ui.row().bind_visibility_from(robot.estop, 'active'):
            ui.icon('report').props('size=md').classes('text-red')
            ui.label('Emergency stop is pressed!').classes('text-red mt-1')

        with ui.row().bind_visibility_from(robot.estop, 'is_soft_estop_active'):
            ui.icon('report').props('size=md').classes('text-red')
            ui.label('Software ESTOP is active!').classes('text-red mt-1')

        with ui.row().bind_visibility_from(robot.estop, 'active', value=False):
            if isinstance(robot.z_axis, Axis):
                with ui.row().bind_visibility_from(robot.z_axis, 'end_t'):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis in end top position, error!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'end_b'):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis in end bottom position, error!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'alarm'):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Z-axis in alarm, warning!').classes('text-orange mt-1')
            if isinstance(robot.y_axis, ChainAxis | Axis):
                with ui.row().bind_visibility_from(robot.y_axis, 'alarm'):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Y-axis in alarm, warning!').classes('text-orange mt-1')

            if isinstance(robot.y_axis, ChainAxis):
                with ui.row().bind_visibility_from(robot.y_axis, 'ref_t', value=False):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Chain-axis not in top position, warning!').classes('text-orange mt-1')

            if isinstance(robot.bumper, rosys.hardware.Bumper):
                with ui.row().bind_visibility_from(robot.bumper, 'active_bumpers'):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Bumper triggered, warning!').classes('text-orange mt-1')

        with ui.row().classes('place-items-center'):
            ui.label('Robot:').style('color: #EDF4FB').classes('font-bold')
            if isinstance(robot, FieldFriendHardware):
                ui.label('real hardware')
            else:
                ui.label('simulated hardware')

        with ui.row().classes('place-items-center'):
            ui.label('Tool:').style('color: #EDF4FB').classes('font-bold')
            ui.label(robot.implement_name)

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row().classes('place-items-center'):
                ui.label('Status Control:').style('color: #EDF4FB').classes('font-bold')
                status_control_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('Battery:').style('color: #EDF4FB').classes('font-bold')
            ui.label().bind_text_from(robot.bms.state, 'last_update',
                                      backward=lambda _: ', '.join([robot.bms.state.short_string,
                                                                    'charging' if robot.bms.state.is_charging else '']))
            if hasattr(robot, 'battery_control') and robot.battery_control is not None:
                ui.label('').tooltip('Battery Box out connectors 1-4') \
                    .bind_text_from(robot.battery_control, 'status',
                                    backward=lambda x: 'Out 1..4 is on' if x else 'Out 1..4 is off')

        with ui.row().classes('place-items-center'):
            ui.label('Axis:').style('color: #EDF4FB').classes('font-bold')
            axis_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('Flashlight:').style('color: #EDF4FB').classes('font-bold')
            if isinstance(robot.flashlight, FlashlightPWMHardware | FlashlightPWMHardwareV2):
                ui.label().bind_text_from(robot.flashlight, 'is_active', backward=lambda x: 'ON' if x else 'OFF')
                ui.label().bind_text_from(robot.flashlight, 'duty_cycle', backward=lambda x: f'{x * 100:.0f}%') \
                    .bind_visibility_from(robot.flashlight, 'is_active')
            else:
                ui.label('simulated')

        if robot.bumper is not None:
            with ui.row().classes('place-items-center'):
                ui.label('Bumper:').style('color: #EDF4FB').classes('font-bold')
                ui.label().bind_text_from(robot.bumper, 'active_bumpers', backward=', '.join)
        if system.is_real and isinstance(robot.wheels, DoubleWheelsHardware):
            with ui.row().classes('place-items-center'):
                ui.label('Motor Status:').style('color: #EDF4FB').classes('font-bold')
                if robot.wheels.odrive_version == 6:
                    ui.label().bind_text_from(robot.wheels, 'l0_error', backward=lambda x: 'Error in l0' if x else 'No error')
                    ui.label().bind_text_from(robot.wheels, 'l1_error', backward=lambda x: 'Error in l1' if x else 'No error')
                    ui.label().bind_text_from(robot.wheels, 'r0_error', backward=lambda x: 'Error in r0' if x else 'No error')
                    ui.label().bind_text_from(robot.wheels, 'r1_error', backward=lambda x: 'Error in r1' if x else 'No error')
                if robot.wheels.odrive_version == 4:
                    ui.label('cant read status update odrive to version 0.5.6')
                ui.button('Reset motor errors', on_click=robot.wheels.reset_motors) \
                    .bind_visibility_from(robot.wheels, 'motor_error')
        if system.is_real and isinstance(robot.z_axis, TornadoHardware):
            with ui.row().classes('place-items-center'):
                ui.label('Tornado motor status:').style('color: #EDF4FB').classes('font-bold')
                if robot.z_axis.odrive_version == 6:
                    ui.label().bind_text_from(robot.z_axis, 'turn_error', backward=lambda x: 'Error in turn motor' if x else 'No error')
                    ui.label().bind_text_from(robot.z_axis, 'z_error', backward=lambda x: 'Error in z motor' if x else 'No error')
                else:
                    ui.label('cant read status update odrive to version 0.5.6')
                ui.button('Reset tornado motor errors', on_click=robot.z_axis.reset_motors) \
                    .bind_visibility_from(robot.z_axis, 'motor_error')

        if robot.mower is not None:
            with ui.row().classes('place-items-center'):
                ui.label('Mower status:').style('color: #EDF4FB').classes('font-bold')
                ui.label().bind_text_from(robot.mower, 'm0_error', backward=lambda x: 'Error in m0' if x else 'No error')
                ui.label().bind_text_from(robot.mower, 'm1_error', backward=lambda x: 'Error in m1' if x else 'No error')
                ui.label().bind_text_from(robot.mower, 'm2_error', backward=lambda x: 'Error in m2' if x else 'No error')
                ui.button('Reset mower errors', on_click=robot.mower.reset_motors) \
                    .bind_visibility_from(robot.mower, 'motor_error')

    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.label('Robot Brain').style('color: #6E93D6;').classes('w-full text-center font-bold')
        ui.separator()

        with ui.row().classes('place-items-center'):
            ui.label('Uptime:').style('color: #EDF4FB').classes('font-bold')
            uptime_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('CPU:').style('color: #EDF4FB').classes('font-bold')
            cpu_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('RAM:').style('color: #EDF4FB').classes('font-bold')
            ram_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('Temperature:').style('color: #EDF4FB').classes('font-bold')
            temperature_label = ui.label()

    def update_status() -> None:
        if isinstance(robot.y_axis, ChainAxis):
            y_axis_flags = [
                'not referenced' if not robot.y_axis.is_referenced else '',
                'alarm' if robot.y_axis.alarm else '',
                'idle'if robot.y_axis.idle else 'moving',
                'ref t' if robot.y_axis.ref_t else '',
                f'{robot.y_axis.steps:.0f}',
                f'{robot.y_axis.position:.2f}m' if robot.y_axis.is_referenced else ''
            ]
        elif isinstance(robot.y_axis, Axis):
            y_axis_flags = [
                'not referenced' if not robot.y_axis.is_referenced else '',
                'alarm' if robot.y_axis.alarm else '',
                'idle'if robot.y_axis.idle else 'moving',
                'end l' if robot.y_axis.end_l else '',
                'end r' if robot.y_axis.end_r else '',
                f'{robot.y_axis.steps:.0f}',
                f'{robot.y_axis.position:.2f}m' if robot.y_axis.is_referenced else ''
            ]
        else:
            y_axis_flags = ['no y-axis']
        if isinstance(robot.z_axis, Axis):
            z_axis_flags = [
                '' if robot.z_axis.is_referenced else 'not referenced',
                'alarm' if robot.z_axis.alarm else '',
                'idle' if robot.z_axis.idle else 'moving',
                'ref_t active' if robot.z_axis.end_t else '',
                'end_b active' if robot.z_axis.end_b else '',
                f'{robot.z_axis.steps}',
                f'{robot.z_axis.position:.2f}m' if robot.z_axis.is_referenced else '',
            ]
        elif isinstance(robot.z_axis, Tornado):
            z_axis_flags = [
                '' if robot.z_axis.is_referenced else 'not referenced',
                '' if robot.z_axis.z_is_referenced else 'z not referenced',
                '' if robot.z_axis.turn_is_referenced else 'turn not referenced',
                'end_top' if robot.z_axis.end_top else '',
                'end_bottom' if robot.z_axis.end_bottom else '',
                'ref_motor' if robot.z_axis.ref_motor else '',
                'ref_gear' if robot.z_axis.ref_gear else '',
                'ref_knife_stop' if robot.z_axis.ref_knife_stop else '',
                'ref_knife_ground' if robot.z_axis.ref_knife_ground else '',
                f'{robot.z_axis.position_z:.2f}m' if robot.z_axis.z_is_referenced else '',
                f'{robot.z_axis.position_turn:.2f}°' if robot.z_axis.turn_is_referenced else '',
            ]
        else:
            z_axis_flags = ['no z-axis']

        y_axis_text = ', '.join(flag for flag in y_axis_flags if flag)
        z_axis_text = ', '.join(flag for flag in z_axis_flags if flag)
        axis_label.text = f'Y-Axis:{y_axis_text}, Z-Axis: {z_axis_text}'

        uptime_label.set_text(f'{timedelta(seconds=rosys.uptime())}')
        cpu_label.set_text(f'{psutil.cpu_percent():.0f}%')
        ram_label.set_text(f'{psutil.virtual_memory().percent:.0f}%')

        if isinstance(robot, FieldFriendHardware):
            temperature_label.set_text(f'{system.get_jetson_cpu_temperature()}°C')

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            status_control_label.text = f'RDYP: {robot.status_control.rdyp_status}, VDP: {robot.status_control.vdp_status}, heap: {robot.status_control.heap}'

    ui.timer(rosys.config.ui_update_interval, update_status)
    return status_dev_page
