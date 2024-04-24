from datetime import timedelta
from typing import TYPE_CHECKING

import psutil
import rosys
from nicegui import ui

from ...hardware import (ChainAxis, FieldFriend, FieldFriendHardware, FlashlightPWMHardware, FlashlightPWMHardwareV2,
                         Tornado, YAxis, YAxisCanOpen, YAxisTornado, ZAxis, ZAxisCanOpen, ZAxisV2)

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
            if isinstance(robot.z_axis, ZAxis) or isinstance(robot.z_axis, ZAxisV2):
                with ui.row().bind_visibility_from(robot.z_axis, 'ref_t', value=False):
                    ui.icon('report').props('size=md').classes('text-red')
                    ui.label('Z-axis not in top position!').classes('text-red mt-1')

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
            ui.markdown('**Robot:**').style('color: #EDF4FB')
            if isinstance(robot, FieldFriendHardware):
                ui.label('real hardware')
            else:
                ui.label('simulated hardware')

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row().classes('place-items-center'):
                ui.markdown('**Status Control:**').style('color: #EDF4FB')
                status_control_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Battery:**').style('color: #EDF4FB')
            bms_label = ui.label()
            if hasattr(robot, 'battery_control'):
                battery_control_label = ui.label('')

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

    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.markdown('**Performance**').style('color: #6E93D6').classes('w-full text-center')
        ui.separator()

        with ui.row().classes('place-items-center'):
            ui.markdown('**Current Field:**').style('color: #EDF4FB')
            current_field_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Current Row:**').style('color: #EDF4FB')
            current_row_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Time on Field:**').style('color: #EDF4FB')
            kpi_fieldtime_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Distance:**').style('color: #EDF4FB')
            kpi_distance_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Processed Rows:**').style('color: #EDF4FB')
            kpi_rows_weeded_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Crops Detected:**').style('color: #EDF4FB')
            kpi_crops_detected_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Weeds Detected:**').style('color: #EDF4FB')
            kpi_weeds_detected_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Punches:**').style('color: #EDF4FB')
            kpi_punches_label = ui.label()

    with ui.card().style('background-color: #3E63A6; color: white;'):
        ui.markdown('**Positioning**').style('color: #6E93D6').classes('w-full text-center')
        ui.separator()
        with ui.row().classes('place-items-center'):
            ui.markdown('**GNSS-Device:**').style('color: #EDF4FB')
            gnss_device_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Reference position:**').style('color: #EDF4FB')
            reference_position_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Position:**').style('color: #EDF4FB')
            gnss_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**Heading:**').style('color: #EDF4FB')
            heading_label = ui.label()
        with ui.row().classes('place-items-center'):
            ui.markdown('**RTK-Fix:**').style('color: #EDF4FB')
            rtk_fix_label = ui.label()

        with ui.row().classes('place-items-center'):
            ui.markdown('**odometry:**').style('color: #EDF4FB')
            odometry_label = ui.label()

    def update_status() -> None:
        bms_flags = [
            f'{robot.bms.state.short_string}',
            'charging' if robot.bms.state.is_charging else ''
        ]
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
                'ref l' if robot.y_axis.end_l else '',
                'ref r' if robot.y_axis.end_r else '',
                f'{robot.y_axis.steps:.0f}',
                f'{robot.y_axis.position:.2f}m' if robot.y_axis.is_referenced else ''
            ]
        elif isinstance(robot.y_axis, YAxisTornado):
            y_axis_flags = [
                'not referenced' if not robot.y_axis.is_referenced else '',
                'alarm' if robot.y_axis.alarm else '',
                'idle'if robot.y_axis.idle else 'moving',
                'end l' if robot.y_axis.end_l else '',
                'end r' if robot.y_axis.end_r else '',
                f'{robot.y_axis.steps:.0f}',
                f'{robot.y_axis.position:.2f}m' if robot.y_axis.is_referenced else ''
            ]
        elif isinstance(robot.y_axis, YAxisCanOpen):
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
        if isinstance(robot.z_axis, ZAxis) or isinstance(robot.z_axis, ZAxisV2):
            z_axis_flags = [
                '' if robot.z_axis.is_referenced else 'not referenced',
                'alarm' if robot.z_axis.alarm else '',
                'idle' if robot.z_axis.idle else 'moving',
                'ref stop enabled' if robot.z_axis.is_ref_enabled else '',
                'end disabled' if not robot.z_axis.is_end_b_enabled else '',
                'ref_t active' if robot.z_axis.ref_t else '',
                'end_b active' if robot.z_axis.end_b else '',
                f'{robot.z_axis.steps}',
                f'{robot.z_axis.depth:.2f}m' if robot.z_axis.is_referenced else '',
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
        elif isinstance(robot.z_axis, ZAxisCanOpen):
            z_axis_flags = [
                '' if robot.z_axis.is_referenced else 'not referenced',
                'alarm' if robot.z_axis.alarm else '',
                'idle' if robot.z_axis.idle else 'moving',
                'end_t' if robot.z_axis.end_t else '',
                'end_b' if robot.z_axis.end_b else '',
                f'{robot.z_axis.steps}',
                f'{robot.z_axis.position:.2f}m' if robot.z_axis.is_referenced else '',
            ]
        else:
            z_axis_flags = ['no z-axis']
        bms_label.text = ', '.join(flag for flag in bms_flags if flag)
        if hasattr(robot, 'battery_control') and robot.battery_control is not None:
            battery_control_label.text = 'Ready' if robot.battery_control.status else 'Not ready'

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
        else:
            bumper_label.text = 'simulated'

        uptime_label.set_text(f'{timedelta(seconds=rosys.uptime())}')
        cpu_label.text = f'{psutil.cpu_percent():.0f}%'
        ram_label.text = f'{psutil.virtual_memory().percent:.0f}%'

        def get_jetson_cpu_temperature():
            with open("/sys/devices/virtual/thermal/thermal_zone0/temp", "r") as f:
                temp = f.read().strip()
            return float(temp) / 1000.0  # Convert from milli°C to °C
        if isinstance(robot, FieldFriendHardware):
            temperature_label.text = f'{get_jetson_cpu_temperature()}°C'

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            status_control_label.text = f'RDYP: {robot.status_control.rdyp_status}, VDP: {robot.status_control.vdp_status}, heap: {robot.status_control.heap}'
        direction_flag = 'N' if system.gnss.record.heading <= 23 else \
            'NE' if system.gnss.record.heading <= 68 else \
            'E' if system.gnss.record.heading <= 113 else \
            'SE' if system.gnss.record.heading <= 158 else \
            'S' if system.gnss.record.heading <= 203 else \
            'SW' if system.gnss.record.heading <= 248 else \
            'W' if system.gnss.record.heading <= 293 else \
            'NW' if system.gnss.record.heading <= 338 else \
            'N'

        if system.automator.is_running:
            if system.field_provider.active_field is not None:
                current_field_label.text = system.field_provider.active_field.name
            kpi_fieldtime_label.text = system.kpi_provider.current_weeding_kpis.time
            kpi_distance_label.text = system.kpi_provider.current_weeding_kpis.distance

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

        gnss_device_label.text = 'No connection' if system.gnss.device is None else 'Connected'
        reference_position_label.text = 'No reference' if system.gnss.reference_lat is None else 'Set'
        gnss_label.text = f'lat: {system.gnss.record.latitude:.6f}, lon: {system.gnss.record.longitude:.6f}'
        heading_label.text = f'{system.gnss.record.heading:.2f}° ' + direction_flag
        rtk_fix_label.text = f'gps_qual: {system.gnss.record.gps_qual}, mode: {system.gnss.record.mode}'
        odometry_label.text = str(system.odometer.prediction)

    ui.timer(rosys.config.ui_update_interval, update_status)
    return status_dev_page
