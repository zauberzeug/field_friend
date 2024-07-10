from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING

import rosys
from nicegui import background_tasks, ui

from .status_bulb import status_bulb

if TYPE_CHECKING:
    from ...system import System

MUTEX_PATH = Path('.livesync_mutex')


class io_sockets:

    def __init__(self, system: System) -> None:
        self.system = system
        if isinstance(system.field_friend, rosys.hardware.RobotHardware):
            self.robot_brain = system.field_friend.robot_brain
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as bumper_card:
            ui.markdown('**Bumper**').classes('w-full text-center')
            ui.separator()
            if self.system.field_friend.bumper is not None:
                with ui.row():
                    if 'front_top' in self.system.field_friend.bumper.active_bumpers:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Front Top Bumper')
                with ui.row():
                    if 'front_bottom' in self.system.field_friend.bumper.active_bumpers:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Front Bottom Bumper')
                with ui.row():
                    if 'back' in self.system.field_friend.bumper.active_bumpers:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Back Bumper')
        if self.system.field_friend.y_axis is not None:
            with ui.card().style('min-width: 200px; background-color: #3E63A6') as y_axis_card:
                ui.markdown('**Y-Axis**').classes('w-full text-center')
                ui.separator()
                with ui.row():
                    if self.system.field_friend.y_axis.alarm:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Alarm')
                with ui.row():
                    if self.system.field_friend.y_axis.end_l:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Left End Switch')
                with ui.row():
                    if self.system.field_friend.y_axis.end_r:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Right End Switch')
                with ui.row():
                    if self.system.field_friend.y_axis.steps:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Step')
                with ui.row():
                    # if self.system.field_friend.y_axis.direction:
                    #     status_bulb(True)
                    # else:
                    #     status_bulb(False)
                    ui.label('Direction')
        if self.system.field_friend.z_axis is not None:
            with ui.card().style('min-width: 200px; background-color: #3E63A6') as z_axis_card:
                ui.markdown('**Z-Axis**').classes('w-full text-center')
                ui.separator()
                with ui.row():
                    if self.system.field_friend.z_axis.end_top:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('End Top Switch')
                with ui.row():
                    if self.system.field_friend.z_axis.end_bottom:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('End Bottom Switch')
                with ui.row():
                    if self.system.field_friend.z_axis.ref_motor:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Ref Motor')
                with ui.row():
                    if self.system.field_friend.z_axis.ref_gear:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Ref Gear')
                with ui.row():
                    if self.system.field_friend.z_axis.ref_knife_stop:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Ref Knife Stop')
                with ui.row():
                    if self.system.field_friend.z_axis.ref_knife_ground:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Ref Knife Ground')
            if self.system.field_friend.wheels is not None:
                with ui.card().style('min-width: 200px; background-color: #3E63A6') as wheels_card:
                    ui.markdown('**Wheels**').classes('w-full text-center')
                    ui.separator()
                    if self.system.is_real:
                        # TODO: can messages testen
                        with ui.row():
                            if self.system.field_friend.wheels:
                                status_bulb(True)
                            else:
                                status_bulb(False)
                            ui.label('Moving')
                    else:
                        with ui.row():
                            if self.system.field_friend.wheels:
                                status_bulb(True)
                            else:
                                status_bulb(False)
                            ui.label('Connected')
            with ui.card().style('min-width: 200px; background-color: #3E63A6') as flashlight_card:
                ui.markdown('**Flashlight**').classes('w-full text-center')
                ui.separator()
                if self.system.field_friend.flashlight is not None:
                    with ui.row():
                        if self.system.field_friend.flashlight.is_active:
                            status_bulb(True)
                        else:
                            status_bulb(False)
                        ui.label('Turned On')
                else:
                    ui.icon("link_off").props("size=lg").style(
                        "display: block; margin-left: auto; margin-right: auto; margin-top: 20px; margin-bottom: 20px;")
        with ui.card().style('min-width: 200px; background-color: #3E63A6') as batterie_control_card:
            ui.markdown('**Battery-Control**').classes('w-full text-center')
            ui.separator()
        if self.system.is_real:
            with ui.card().style('min-width: 200px; background-color: #3E63A6') as batterie_control_card:
                ui.markdown('**Expander**').classes('w-full text-center')
                ui.separator()
                with ui.row():
                    # TODO add check if the enable pin of expander is active
                    if True:
                        status_bulb(True)
                    else:
                        status_bulb(False)
                    ui.label('Enabled')

    def set_modules(self) -> None:
        self.module_row.clear()
        with self.module_row:
            ui.label('Get Modules')
