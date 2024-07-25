from __future__ import annotations

from typing import TYPE_CHECKING

from nicegui import ui

from .status_bulb import StatusBulb as status_bulb

if TYPE_CHECKING:
    from ...system import System


class io_overview:
    def __init__(self, system: System) -> None:
        self.system = system
        with ui.card().style('background-color: #2E5396; color: white;'), ui.row():
            with ui.row().style('width: 100%'):
                ui.label('I/O Overview').style('font-size: 1.5rem; color: white;')
            with ui.card().style('background-color: #3E63A6;'):
                ui.markdown('**E-Stops**').classes('w-full text-center')
                ui.separator()
                with ui.row():
                    status_bulb().bind_value_from(system.field_friend.estop, 'is_soft_estop_active')
                    ui.label('Soft E-Stop')
                if system.is_real:
                    with ui.row():
                        status_bulb().bind_value_from(system.field_friend.estop, 'pressed_estops', lambda pressed_estops: 0 in pressed_estops)
                        ui.label('Hard E-Stop 0')
                    with ui.row():
                        status_bulb().bind_value_from(system.field_friend.estop, 'pressed_estops', lambda pressed_estops: 1 in pressed_estops)
                        ui.label('Hard E-Stop 1')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Bumper**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.bumper is not None:
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.bumper, 'active_bumpers',
                                                      lambda active_bumpers: 'front_top' in active_bumpers)
                        ui.label('Front Top Bumper')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.bumper, 'active_bumpers',
                                                      lambda active_bumpers: 'front_bottom' in active_bumpers)
                        ui.label('Front Bottom Bumper')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.bumper, 'active_bumpers',
                                                      lambda active_bumpers: 'back' in active_bumpers)
                        ui.label('Back Bumper')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Y-Axis**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.y_axis is not None:
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.y_axis, 'alarm')
                        ui.label('Alarm')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.y_axis, 'end_l')
                        ui.label('Left End Switch')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.y_axis, 'end_r')
                        ui.label('Right End Switch')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Z-Axis**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.z_axis is not None:
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'end_top')
                        ui.label('End Top Switch')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'end_bottom')
                        ui.label('End Bottom Switch')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'ref_motor')
                        ui.label('Ref Motor')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'ref_gear')
                        ui.label('Ref Gear')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'ref_knife_stop')
                        ui.label('Ref Knife Stop')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.z_axis, 'ref_knife_ground')
                        ui.label('Ref Knife Ground')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Wheels**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.wheels is not None:
                    if self.system.is_real:
                        with ui.row():
                            print(self.system.field_friend.wheels.linear_target_speed)
                            status_bulb().bind_value_from(self.system.field_friend.wheels, 'linear_target_speed', lambda x: x > 0 or x < 0)
                            ui.label('Forward/Backwards')
                        with ui.row():
                            status_bulb().bind_value_from(self.system.field_friend.wheels, 'angular_target_speed', lambda x: x > 0 or x < 0)
                            ui.label('Turning')
                    else:
                        with ui.row():
                            if self.system.field_friend.wheels:
                                status_bulb().bind_value_from(self.system.field_friend.wheels)
                            else:
                                status_bulb(False)
                            ui.label('Connected')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Flashlight**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.flashlight is not None:
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.flashlight, 'is_active')
                        ui.label('Turned On')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
            with ui.card().style('min-width: 200px; background-color: #3E63A6'):
                ui.markdown('**Battery**').classes('w-full text-center')
                ui.separator()
                if self.system.is_real:
                    with ui.row():
                        status_bulb().bind_value_from(
                            self.system.field_friend.bms.state, 'percentage', lambda x: x < 20 if x else False)
                        ui.label('Battery Low (< 20%)')
                    with ui.row():
                        status_bulb().bind_value_from(self.system.field_friend.bms.state, 'is_charging')
                        ui.label('Is Charging')
                else:
                    ui.icon('link_off').props('size=lg').style(
                        'display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;')
