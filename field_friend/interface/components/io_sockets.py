from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

from nicegui import ui

from .status_bulb import StatusBulb as status_bulb

if TYPE_CHECKING:
    from ...system import System

MUTEX_PATH = Path('.livesync_mutex')


class io_sockets:

    def __init__(self, system: System) -> None:
        self.system = system
        with ui.card().style('background-color: #3E63A6;') as estop_card:
            ui.markdown('**E-Stops**').classes('w-full text-center')
            ui.separator()
            with ui.row():
                status_bulb(True).bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active')
                status_bulb(False).bind_visibility_from(
                    system.field_friend.estop, 'is_soft_estop_active', lambda x: not x)
                ui.label('Soft E-Stop')
            if system.is_real:
                with ui.row():
                    status_bulb(True).bind_visibility_from(system.field_friend.estop,
                                                           'pressed_estops', lambda pressed_estops: 0 in pressed_estops)
                    status_bulb(False).bind_visibility_from(
                        system.field_friend.estop, 'pressed_estops', lambda pressed_estops: 0 not in pressed_estops)
                    ui.label('Hard E-Stop 0')
                with ui.row():
                    status_bulb(True).bind_visibility_from(system.field_friend.estop,
                                                           'pressed_estops', lambda pressed_estops: 1 in pressed_estops)
                    status_bulb(False).bind_visibility_from(system.field_friend.estop,
                                                            'pressed_estops', lambda pressed_estops: 1 not in pressed_estops)
                    ui.label('Hard E-Stop 1')
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as bumper_card:
            ui.markdown('**Bumper**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.bumper is not None:
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                           lambda active_bumpers: True if 'front_top' in active_bumpers else False)
                    status_bulb(False).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                            lambda active_bumpers: not (True if 'front_top' in active_bumpers else False))
                    ui.label('Front Top Bumper')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                           lambda active_bumpers: True if 'front_bottom' in active_bumpers else False)
                    status_bulb(False).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                            lambda active_bumpers: not (True if 'front_bottom' in active_bumpers else False))
                    ui.label('Front Bottom Bumper')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                           lambda active_bumpers: True if 'back' in active_bumpers else False)
                    status_bulb(False).bind_visibility_from(self.system.field_friend.bumper, 'active_bumpers',
                                                            lambda active_bumpers: not (True if 'back' in active_bumpers else False))
                    ui.label('Back Bumper')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as y_axis_card:
            ui.markdown('**Y-Axis**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.y_axis is not None:
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.y_axis, 'alarm')
                    status_bulb(False).bind_visibility_from(self.system.field_friend.y_axis, 'alarm', lambda x: not x)
                    ui.label('Alarm')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.y_axis, 'end_l')
                    status_bulb(False).bind_visibility_from(self.system.field_friend.y_axis, 'end_l', lambda x: not x)
                    ui.label('Left End Switch')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.y_axis, 'end_r')
                    status_bulb(False).bind_visibility_from(self.system.field_friend.y_axis, 'end_r', lambda x: not x)
                    ui.label('Right End Switch')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as z_axis_card:
            ui.markdown('**Z-Axis**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.z_axis is not None:
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'end_top')
                    status_bulb(False).bind_visibility_from(self.system.field_friend.z_axis, 'end_top', lambda x: not x)
                    ui.label('End Top Switch')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'end_bottom')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.z_axis, 'end_bottom', lambda x: not x)
                    ui.label('End Bottom Switch')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'ref_motor')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.z_axis, 'ref_motor', lambda x: not x)
                    ui.label('Ref Motor')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'ref_gear')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.z_axis, 'ref_gear', lambda x: not x)
                    ui.label('Ref Gear')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'ref_knife_stop')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.z_axis, 'ref_knife_stop', lambda x: not x)
                    ui.label('Ref Knife Stop')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.z_axis, 'ref_knife_ground')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.z_axis, 'ref_knife_ground', lambda x: not x)
                    ui.label('Ref Knife Ground')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as wheels_card:
            ui.markdown('**Wheels**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.wheels is not None:
                if self.system.is_real:
                    with ui.row():
                        print(self.system.field_friend.wheels.linear_target_speed)
                        status_bulb(True).bind_visibility_from(
                            self.system.field_friend.wheels, 'linear_target_speed', lambda x: x > 0 or x < 0)
                        status_bulb(False).bind_visibility_from(
                            self.system.field_friend.wheels, 'linear_target_speed', lambda x: x == 0)
                        ui.label('Forward/Backwards')
                    with ui.row():
                        status_bulb(True).bind_visibility_from(
                            self.system.field_friend.wheels, 'angular_target_speed', lambda x: x > 0 or x < 0)
                        status_bulb(False).bind_visibility_from(
                            self.system.field_friend.wheels, 'angular_target_speed', lambda x: x == 0)
                        ui.label('Turning')
                else:
                    with ui.row():
                        if self.system.field_friend.wheels:
                            status_bulb(True).bind_visibility_from(self.system.field_friend.wheels)
                        else:
                            status_bulb(False)
                        ui.label('Connected')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as flashlight_card:
            ui.markdown('**Flashlight**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.flashlight is not None:
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.flashlight, 'is_active')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.flashlight, 'is_active', lambda x: not x)
                    ui.label('Turned On')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as batterie_control_card:
            ui.markdown('**Battery**').classes('w-full text-center')
            ui.separator()
            if self.system.is_real:
                with ui.row():
                    status_bulb(True).bind_visibility_from(
                        self.system.field_friend.bms.state, 'percentage', lambda x: x < 20 if x else False)
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.bms.state, 'percentage', lambda x: x >= 20 if x else False)
                    ui.label('Battery Low (< 20%)')
                with ui.row():
                    status_bulb(True).bind_visibility_from(self.system.field_friend.bms.state, 'is_charging')
                    status_bulb(False).bind_visibility_from(
                        self.system.field_friend.bms.state, 'is_charging', lambda x: not x)
                    ui.label('Is Charging')
            else:
                ui.icon("link_off").props("size=lg").style(
                    "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
