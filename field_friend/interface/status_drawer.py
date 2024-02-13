from datetime import timedelta

import psutil
import rosys
from nicegui import ui

from ..hardware import (ChainAxis, FieldFriend, FieldFriendHardware, FlashlightPWMHardware, Tornado, YAxis,
                        YAxisTornado, ZAxis, ZAxisV2)
from ..navigation import Gnss


def status_drawer(robot: FieldFriend, gnss: Gnss, odometer: rosys.driving.Odometer):
    with ui.right_drawer().classes('bg-[#edf4fa]') as status_drawer, ui.column():
        ui.label('System status').classes('text-xl')
        ui.markdown('**Hardware:**')

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

        with ui.row():
            ui.markdown('**Robot:**').style('color: #6E93D6')
            if isinstance(robot, FieldFriendHardware):
                ui.label('real hardware')
            else:
                ui.label('simulated hardware')

        if hasattr(robot, 'status_control') and robot.status_control is not None:
            with ui.row():
                ui.markdown('**Status Control:**').style('color: #6E93D6')
                status_control_label = ui.label()

        with ui.row():
            ui.markdown('**Battery:**').style('color: #6E93D6')
            bms_label = ui.label()
            if hasattr(robot, 'battery_control'):
                battery_control_label = ui.label('')

        with ui.row():
            ui.markdown('**Y-Axis:**').style('color: #6E93D6')
            y_axis_label = ui.label()

        with ui.row():
            ui.markdown('**Z-Axis:**').style('color: #6E93D6')
            z_axis_label = ui.label()

        with ui.row():
            ui.markdown('**Flashlight:**').style('color: #6E93D6')
            flashlight_label = ui.label()

        if hasattr(robot, 'bumper') and robot.bumper is not None:
            with ui.row():
                ui.markdown('**Bumper:**').style('color: #6E93D6')
                bumper_label = ui.label()

        ui.markdown('**Robot Brain:**')
        with ui.row():
            ui.markdown('**Uptime:**').style('color: #6E93D6')
            uptime_label = ui.label()

        with ui.row():
            ui.markdown('**CPU:**').style('color: #6E93D6')
            cpu_label = ui.label()

        with ui.row():
            ui.markdown('**RAM:**').style('color: #6E93D6')
            ram_label = ui.label()

        with ui.row():
            ui.markdown('**Temperature:**').style('color: #6E93D6')
            temperature_label = ui.label()

        ui.markdown('**Positioning:**')

        with ui.row():
            ui.markdown('**GNSS-Device:**').style('color: #6E93D6')
            gnss_device_label = ui.label()
        with ui.row():
            ui.markdown('**Reference position:**').style('color: #6E93D6')
            reference_position_label = ui.label()
        with ui.row():
            ui.markdown('**Position:**').style('color: #6E93D6')
            gnss_label = ui.label()
        with ui.row():
            ui.markdown('**Heading:**').style('color: #6E93D6')
            heading_label = ui.label()
        with ui.row():
            ui.markdown('**RTK-Fix:**').style('color: #6E93D6')
            rtk_fix_label = ui.label()

        with ui.row():
            ui.markdown('**odometry:**').style('color: #6E93D6')
            odometry_label = ui.label()

        with ui.row():
            ui.markdown('**imu:**').style('color: #6E93D6')
            imu_label = ui.label()

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
                    'ref_t' if robot.z_axis.ref_t else '',
                    'ref_b' if robot.z_axis.ref_b else '',
                    f'{robot.z_axis.position_z:.2f}m' if robot.z_axis.z_is_referenced else '',
                    f'{robot.z_axis.position_turn:.2f}°' if robot.z_axis.turn_is_referenced else '',
                ]
            else:
                z_axis_flags = ['no z-axis']
            bms_label.text = ', '.join(flag for flag in bms_flags if flag)
            if hasattr(robot, 'battery_control') and robot.battery_control is not None:
                battery_control_label.text = 'Ready' if robot.battery_control.status else 'Not ready'

            y_axis_label.text = ', '.join(flag for flag in y_axis_flags if flag)
            z_axis_label.text = ', '.join(flag for flag in z_axis_flags if flag)
            if isinstance(robot.flashlight, FlashlightPWMHardware):
                flashlight_label.text = f'{robot.flashlight.duty_cycle * 100:.0f}%'
            if isinstance(robot.bumper, rosys.hardware.Bumper):
                bumper_label.text = ', '.join(robot.bumper.active_bumpers)

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
            direction_flag = 'N' if gnss.record.heading <= 23 else \
                'NE' if gnss.record.heading <= 68 else \
                'E' if gnss.record.heading <= 113 else \
                'SE' if gnss.record.heading <= 158 else \
                'S' if gnss.record.heading <= 203 else \
                'SW' if gnss.record.heading <= 248 else \
                'W' if gnss.record.heading <= 293 else \
                'NW' if gnss.record.heading <= 338 else \
                'N'
            gnss_device_label.text = 'No connection' if gnss.device is None else 'Connected'
            reference_position_label.text = 'No reference' if gnss.reference_lat is None else 'Set'
            gnss_label.text = f'lat: {gnss.record.latitude:.6f}, lon: {gnss.record.longitude:.6f}'
            heading_label.text = f'{gnss.record.heading:.2f}° ' + direction_flag
            rtk_fix_label.text = f'gps_qual: {gnss.record.gps_qual}, mode: {gnss.record.mode}'
            odometry_label.text = str(odometer.prediction)
            imu_label.text = ('y:'+str(round(robot.imu.yaw, 1)) + '°, p:' +
                              str(round(robot.imu.pitch, 1)) + '°, r:' + str(round(robot.imu.roll, 1))+'°')

        ui.timer(rosys.config.ui_update_interval, update_status)
    return status_drawer
