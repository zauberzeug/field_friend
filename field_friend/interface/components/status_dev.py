from datetime import timedelta
from typing import TYPE_CHECKING

import psutil
import rosys
from nicegui import ui

from ... import localization
from ...hardware import (
    Axis,
    AxisD1,
    ChainAxis,
    FieldFriend,
    FieldFriendHardware,
    FlashlightPWMHardware,
    FlashlightPWMHardwareV2,
    Tornado,
)

if TYPE_CHECKING:
    from field_friend.system import System


def status_dev_page(robot: FieldFriend, system: 'System'):
    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.markdown('**Hardware**').style('color: #6E93D6;').classes('w-full text-center')
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
            if isinstance(robot.y_axis, ChainAxis) or isinstance(robot.y_axis, Axis):
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
            ui.markdown('**Robot:**').style('color: #EDF4FB')
            if isinstance(robot, FieldFriendHardware):
                ui.label('real hardware')
            else:
                ui.label('simulated hardware')

        with ui.row().classes('place-items-center'):
            ui.markdown('**Tool:**').style('color: #EDF4FB')
            ui.label(robot.implement_name)

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Status Control:**').style('color: #EDF4FB')
                status_control_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Axis:**').style('color: #EDF4FB')
            axis_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Flashlight:**').style('color: #EDF4FB')
            flashlight_label = ui.label()

        if hasattr(robot, 'bumper') and robot.bumper is not None:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Bumper:**').style('color: #EDF4FB')
                bumper_label = ui.label()
        if system.is_real:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Motor status:**').style('color: #EDF4FB')
                l0_status = ui.label()
                l1_status = ui.label()
                r0_status = ui.label()
                r1_status = ui.label()
                reset_motor_button = ui.button(
                    'Reset motor errors', on_click=robot.wheels.reset_motors).bind_visibility_from(robot.wheels, 'motor_error')
        if system.is_real and isinstance(robot.z_axis, Tornado):
            with ui.row().classes('place-items-center'):
                ui.markdown('**Tornado motor status:**').style('color: #EDF4FB')
                tornado_motor_turn_status = ui.label()
                tornado_motor_z_status = ui.label()
                reset_tornado_motor_button = ui.button(
                    'Reset tornado motor errors', on_click=robot.z_axis.reset_motors).bind_visibility_from(robot.z_axis, 'motor_error')

        if hasattr(robot, 'mower') and robot.mower is not None:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Mower status:**').style('color: #EDF4FB')
                m0_status = ui.label()
                m1_status = ui.label()
                m2_status = ui.label()
                reset_mower_button = ui.button(
                    'Reset mower errors', on_click=robot.mower.reset_motors).bind_visibility_from(robot.mower, 'motor_error')

    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.markdown('**Robot Brain**').style('color: #6E93D6;').classes('w-full text-center')
        ui.separator()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Uptime:**').style('color: #EDF4FB')
            uptime_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**CPU:**').style('color: #EDF4FB')
            cpu_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**RAM:**').style('color: #EDF4FB')
            ram_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Temperature:**').style('color: #EDF4FB')
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

        if isinstance(robot.flashlight, FlashlightPWMHardware):
            flashlight_label.text = f'{robot.flashlight.duty_cycle * 100:.0f}%'
        elif isinstance(robot.flashlight, FlashlightPWMHardwareV2):
            flashlight_label.text = f'{"ON" if robot.flashlight.is_active else "OFF"}  {f"at {robot.flashlight.duty_cycle * 100:.0f}%" if robot.flashlight.is_active else ""}'
        else:
            flashlight_label.text = 'simulated'
        if isinstance(robot.bumper, rosys.hardware.Bumper):
            bumper_label.text = ', '.join(robot.bumper.active_bumpers)

        uptime_label.set_text(f'{timedelta(seconds=rosys.uptime())}')
        cpu_label.text = f'{psutil.cpu_percent():.0f}%'
        ram_label.text = f'{psutil.virtual_memory().percent:.0f}%'

        if isinstance(robot, FieldFriendHardware):
            temperature_label.text = f'{system.get_jetson_cpu_temperature()}°C'

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            status_control_label.text = f'RDYP: {robot.status_control.rdyp_status}, VDP: {robot.status_control.vdp_status}, heap: {robot.status_control.heap}'
        if system.is_real:
            if robot.wheels.odrive_version == 6:
                l0_status.text = 'Error in l0' if robot.wheels.l0_error else 'No error'
                l1_status.text = 'Error in l1' if robot.wheels.l1_error else 'No error'
                r0_status.text = 'Error in r0' if robot.wheels.r0_error else 'No error'
                r1_status.text = 'Error in r1' if robot.wheels.r1_error else 'No error'
            if robot.wheels.odrive_version == 4:
                l0_status.text = 'cant read status update odrive to version 0.5.6'
        if system.is_real and isinstance(robot.z_axis, Tornado):
            if robot.z_axis.odrive_version == 6:
                tornado_motor_turn_status.text = 'Error in turn motor' if robot.z_axis.turn_error else 'No error'
                tornado_motor_z_status.text = 'Error in z motor' if robot.z_axis.z_error else 'No error'
            else:
                tornado_motor_turn_status.text = 'cant read status update odrive to version 0.5.6'

        if hasattr(robot, 'mower') and robot.mower is not None:
            m0_status.text = 'Error in m0' if robot.mower.m0_error else 'No error'
            m1_status.text = 'Error in m1' if robot.mower.m1_error else 'No error'
            m2_status.text = 'Error in m2' if robot.mower.m2_error else 'No error'

    ui.timer(rosys.config.ui_update_interval, update_status)
    return status_dev_page
