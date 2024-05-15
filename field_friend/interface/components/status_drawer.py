from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from ...hardware import (
    ChainAxis,
    FieldFriend,
    FlashlightPWMHardware,
    FlashlightPWMHardwareV2,
    Tornado,
    YAxis,
    ZAxis,
)
from ...navigation import Gnss

if TYPE_CHECKING:
    from field_friend.system import System


def status_drawer(system: 'System', robot: FieldFriend, gnss: Gnss, odometer: rosys.driving.Odometer, automator: rosys.automation.Automator):
    with ui.right_drawer(value=False).classes('bg-[#edf4fa]') as status_drawer, ui.column():
        with ui.row().classes('w-full place-content-end'):
            info_dialog = system.info.create_dialog()
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
                        ui.menu_item('Restart Lizard', on_click=system.field_friend.robot_brain.restart)
                    ui.menu_item('Clear GNSS reference', on_click=system.gnss.clear_reference)

        ui.label('System Status').classes('text-xl')

        ui.markdown('**Hardware**').style('color: #6E93D6').classes('w-full text-center')
        ui.separator()

        with ui.row().bind_visibility_from(robot.estop, 'active'):
            ui.icon('report').props('size=md').classes('text-red')
            ui.label('Emergency stop is pressed!').classes('text-red mt-1')

        with ui.row().bind_visibility_from(robot.estop, 'is_soft_estop_active'):
            ui.icon('report').props('size=md').classes('text-red')
            ui.label('Software ESTOP is active!').classes('text-red mt-1')

        with ui.row().bind_visibility_from(robot.estop, 'active', value=False):
            if isinstance(robot.z_axis, ZAxis):
                with ui.row().bind_visibility_from(robot.z_axis, 'end_t'):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis in top end position!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'end_b'):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis in end bottom pisition, error!').classes('text-red mt-1')

                with ui.row().bind_visibility_from(robot.z_axis, 'alarm'):
                    ui.icon('report').props('size=md').classes('text-yellow')
                    ui.label('Z-axis in alarm, warning!').classes('text-orange mt-1')
            if isinstance(robot.y_axis, ChainAxis) or isinstance(robot.y_axis, YAxis):
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
                        ui.markdown('**Bumper:**').style('color: #6E93D6')
                        bumper_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Tool:**').style('color: #6E93D6')
            ui.label(robot.tool)

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Status Control:**').style('color: #6E93D6')
                status_control_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Flashlight:**').style('color: #6E93D6')
            flashlight_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Axis:**').style('color: #6E93D6')
            axis_label = ui.label()

        ui.markdown('**Performance**').style('color: #6E93D6').classes('w-full text-center')
        ui.separator()
        with ui.column().bind_visibility_from(system.automator, 'is_running'):
            with ui.row().classes('place-items-center'):
                ui.markdown('**Current Field:**').style('color: #6E93D6')
            with ui.row().classes('place-items-center'):
                ui.markdown('**Time on Field:**').style('color: #6E93D6')
                kpi_fieldtime_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Distance:**').style('color: #6E93D6')
                kpi_distance_label = ui.label()
            current_field_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Current Row:**').style('color: #6E93D6')
                current_row_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Processed Rows:**').style('color: #6E93D6')
                kpi_rows_weeded_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Crops Detected:**').style('color: #6E93D6')
                kpi_crops_detected_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Weeds Detected:**').style('color: #6E93D6')
                kpi_weeds_detected_label = ui.label()
            with ui.row().classes('place-items-center'):
                ui.markdown('**Punches:**').style('color: #6E93D6')
                kpi_punches_label = ui.label()

        with ui.row().classes('place-items-center').bind_visibility_from(system.automator, 'is_running', backward=lambda x: not x):
            ui.markdown('**No automation running**').style('color: #6E93D6')
        ui.markdown('**Positioning**').style('color: #6E93D6').classes('w-full text-center')
        ui.separator()

        with ui.row().classes('place-items-center'):
            ui.markdown('**GNSS-Device:**').style('color: #6E93D6')
            gnss_device_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Reference position:**').style('color: #6E93D6')
            reference_position_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Position:**').style('color: #6E93D6')
            gnss_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Heading:**').style('color: #6E93D6')
            heading_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**RTK-Fix:**').style('color: #6E93D6')
            rtk_fix_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**odometry:**').style('color: #6E93D6')
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
            elif isinstance(robot.y_axis, YAxis):
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
            if isinstance(robot.z_axis, ZAxis):
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

            if isinstance(robot.flashlight, FlashlightPWMHardware) or isinstance(robot.flashlight, FlashlightPWMHardwareV2):
                flashlight_label.text = f'{"On" if robot.flashlight.is_active else "Off"}  {f"at {robot.flashlight.duty_cycle * 100:.0f}%" if robot.flashlight.is_active else ""}'
            else:
                flashlight_label.text = 'simulated'
            if isinstance(robot.bumper, rosys.hardware.Bumper):
                bumper_label.text = ', '.join(robot.bumper.active_bumpers)

            if hasattr(robot, 'status_control') and robot.status_control is not None:
                status_control_label.text = f'RDYP: {robot.status_control.rdyp_status}, VDP: {robot.status_control.vdp_status}, heap: {robot.status_control.heap}'
            direction_flag = 'N' if gnss.record.heading <= 23 else \
                'NE' if gnss.record.heading <= 68 else \
                'E' if gnss.record.heading <= 113 else \
                'SE' if gnss.record.heading <= 158 else \
                'S' if gnss.record.heading <= 203 else \
                'SW' if gnss.record.heading <= 248 else \
                'W' if gnss.record.heading <= 293 else \
                'NW' if gnss.record.heading <= 338 else \
                'N'

            if automator.is_running:
                if system.field_provider.active_field is not None:
                    current_field_label.text = system.field_provider.active_field.name
                kpi_fieldtime_label.text = f'{system.kpi_provider.current_weeding_kpis.time}s'
                kpi_distance_label.text = f'{system.kpi_provider.current_weeding_kpis.distance}m'

                current_automation = next(key for key, value in system.automations.items()
                                          if value == system.automator.default_automation)
                if current_automation == 'weeding' or current_automation == 'monitoring':
                    if system.field_provider.active_object is not None and system.field_provider.active_object['object'] is not None:
                        current_row_label.text = system.field_provider.active_object['object'].name
                    kpi_weeds_detected_label.text = system.kpi_provider.current_weeding_kpis.weeds_detected
                    kpi_crops_detected_label.text = system.kpi_provider.current_weeding_kpis.crops_detected
                    kpi_rows_weeded_label.text = system.kpi_provider.current_weeding_kpis.rows_weeded
                    if current_automation == 'weeding':
                        kpi_punches_label.text = system.kpi_provider.current_weeding_kpis.punches

            gnss_device_label.text = 'No connection' if gnss.device is None else 'Connected'
            reference_position_label.text = 'No reference' if gnss.reference is None else 'Set'
            gnss_label.text = f'lat: {gnss.record.latitude:.6f}, lon: {gnss.record.longitude:.6f}'
            heading_label.text = f'{gnss.record.heading:.2f}° ' + direction_flag
            rtk_fix_label.text = f'gps_qual: {gnss.record.gps_qual}, mode: {gnss.record.mode}'
            odometry_label.text = str(odometer.prediction)

        ui.timer(rosys.config.ui_update_interval, update_status)
    return status_drawer
