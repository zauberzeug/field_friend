# pylint: disable=duplicate-code
# TODO: refactor this and status_dev.py
from __future__ import annotations

from typing import TYPE_CHECKING, cast

import rosys
from nicegui import ui

from ... import localization
from ...hardware import Axis, ChainAxis, FieldFriendHardware, FlashlightPWMHardware, FlashlightPWMHardwareV2, Tornado

if TYPE_CHECKING:
    from ...system import System


def create_status_drawer(system: System) -> ui.right_drawer:
    robot = system.field_friend
    gnss = system.gnss
    odometer = system.odometer
    with ui.right_drawer(value=False).classes('bg-[#edf4fa]') as status_drawer, ui.column():
        with ui.row().classes('w-full place-content-end'):
            # TODO: remove type ignore, not clear why mypy doesn't see that info is of type Info
            info_dialog = system.info.create_dialog()  # type: ignore
            ui.button(on_click=info_dialog.open).props('icon=info flat dense').style('color: #6E93D6;')
            with ui.button().props('icon=settings flat').style('color: #6E93D6'):
                with ui.menu().props(remove='no-parent-event'):
                    with ui.column().classes('gap-0'):
                        rosys.persistence.export_button() \
                            .props('flat align=left').classes('w-full')
                        rosys.persistence.import_button(after_import=system.restart) \
                            .props('flat align=left').classes('w-full')
                    ui.separator()
                    ui.menu_item('Restart RoSys', on_click=system.restart)
                    if system.is_real:
                        ui.menu_item('Restart Lizard', on_click=cast(FieldFriendHardware, robot).robot_brain.restart)

        ui.label('System Status').classes('text-xl')

        ui.label('Hardware').style('color: #6E93D6').classes('w-full text-center font-bold')
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
                    ui.label('Z-axis in top end position!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'end_b'):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis in end bottom position, error!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'alarm'):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Z-axis in alarm, warning!').classes('text-orange mt-1')
            if isinstance(robot.y_axis, Axis | ChainAxis):
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
                        ui.label('Bumper:').style('color: #6E93D6').classes('font-bold')
                        bumper_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Tool:').style('color: #6E93D6')
            ui.label(robot.implement_name)

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row().classes('place-items-center'):
                ui.label('Status Control:').style('color: #6E93D6').classes('font-bold')
                status_control_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('Flashlight:').style('color: #6E93D6').classes('font-bold')
            flashlight_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('Axis:').style('color: #6E93D6').classes('font-bold')
            axis_label = ui.label()

        ui.label('Performance').style('color: #6E93D6').classes('w-full text-center font-bold')
        ui.separator()

        with ui.row().classes('place-items-center').bind_visibility_from(system.automator, 'is_running', backward=lambda x: not x):
            ui.label('No automation running').style('color: #6E93D6').classes('font-bold')
        with ui.row().classes('place-items-center'):
            ui.label('Time in Automation').style('color: #6E93D6').classes('font-bold')
            kpi_time_in_automation_off = ui.label()

        ui.label('Positioning').style('color: #6E93D6').classes('w-full text-center font-bold')
        ui.separator()

        with ui.row().classes('place-items-center'):
            ui.label('GNSS-Device:').style('color: #6E93D6').classes('font-bold')
            gnss_device_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Reference position:').style('color: #6E93D6').classes('font-bold')
            reference_position_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Position:').style('color: #6E93D6').classes('font-bold')
            gnss_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('Heading:').style('color: #6E93D6').classes('font-bold')
            heading_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.label('RTK-Fix:').style('color: #6E93D6').classes('font-bold')
            rtk_fix_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.label('odometry:').style('color: #6E93D6').classes('font-bold')
            odometry_label = ui.label()

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
                    'end_t' if robot.z_axis.end_t else '',
                    'end_b' if robot.z_axis.end_b else '',
                    f'{robot.z_axis.steps}',
                    f'{robot.z_axis.position:.2f}m' if robot.z_axis.is_referenced else '',
                ]
            elif isinstance(robot.z_axis, Tornado):
                z_axis_flags = [
                    '' if robot.z_axis.is_referenced else 'not referenced',
                    '' if robot.z_axis.z_is_referenced else 'z not referenced',
                    '' if robot.z_axis.turn_is_referenced else 'turn not referenced',
                    'top' if robot.z_axis.end_top else '',
                    'bottom' if robot.z_axis.end_bottom else '',
                    'ref_motor' if robot.z_axis.ref_motor else '',
                    'ref_gear' if robot.z_axis.ref_gear else '',
                    'knife_stop' if robot.z_axis.ref_knife_stop else '',
                    'knife_ground' if robot.z_axis.ref_knife_ground else '',
                    f'{robot.z_axis.position_z:.2f}m' if robot.z_axis.z_is_referenced else '',
                    f'{robot.z_axis.position_turn:.2f}°' if robot.z_axis.turn_is_referenced else '',
                ]
            else:
                z_axis_flags = ['no z-axis']

            y_axis_text = ', '.join(flag for flag in y_axis_flags if flag)
            z_axis_text = ', '.join(flag for flag in z_axis_flags if flag)
            axis_label.text = f'Y-AXIS: {y_axis_text} | Z-AXIS: {z_axis_text}'

            if isinstance(robot.flashlight, FlashlightPWMHardware | FlashlightPWMHardwareV2):
                flashlight_label.text = f'{"On" if robot.flashlight.is_active else "Off"}  {f"at {robot.flashlight.duty_cycle * 100:.0f}%" if robot.flashlight.is_active else ""}'
            else:
                flashlight_label.text = 'simulated'
            if isinstance(robot.bumper, rosys.hardware.Bumper):
                bumper_label.text = ', '.join(robot.bumper.active_bumpers)

            if hasattr(robot, 'status_control') and robot.status_control is not None:
                status_control_label.text = f'RDYP: {robot.status_control.rdyp_status}, VDP: {robot.status_control.vdp_status}, heap: {robot.status_control.heap}'
            # TODO: move this into gnss since it is used multiple times, check stuff above this too!
            direction_flag = '?' if gnss.current is None or gnss.current.heading is None else \
                'N' if gnss.current.heading <= 23 else \
                'NE' if gnss.current.heading <= 68 else \
                'E' if gnss.current.heading <= 113 else \
                'SE' if gnss.current.heading <= 158 else \
                'S' if gnss.current.heading <= 203 else \
                'SW' if gnss.current.heading <= 248 else \
                'W' if gnss.current.heading <= 293 else \
                'NW' if gnss.current.heading <= 338 else \
                'N'

            kpi_time_in_automation_off.text = f'{system.kpi_provider.get_time_kpi()}'
            gnss_device_label.text = 'No connection' if gnss.device is None else 'Connected'
            reference_position_label.text = 'No reference' if localization.reference is None else 'Set'
            gnss_label.text = str(system.gnss.current.location) if system.gnss.current is not None else 'No position'
            heading_label.text = f'{system.gnss.current.heading:.2f}° {direction_flag}' if system.gnss.current is not None and system.gnss.current.heading is not None else 'No heading'
            rtk_fix_label.text = f'gps_qual: {system.gnss.current.gps_qual}, mode: {system.gnss.current.mode}' if system.gnss.current is not None else 'No fix'
            odometry_label.text = str(odometer.prediction)
        ui.timer(rosys.config.ui_update_interval, update_status)
    return status_drawer
