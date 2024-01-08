
import logging
from typing import TYPE_CHECKING

import rosys
from nicegui import events, ui

from .automation_controls import automation_controls
from .field_object import field_object
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object
from .visualizer_object import visualizer_object
from ..automations import rolling

if TYPE_CHECKING:
    from field_friend.system import System

SHORTCUT_INFO = '''
    Steer the robot manually with the JOYSTICK on the left. <br>
    Or hold SHIFT and use the ARROW KEYS
'''


class operation:

    def __init__(self, system: 'System') -> None:
        self.log = logging.getLogger('field_friend.operation')
        self.system = system
        self.field = None
        with ui.card().tight():
            self.scene_look = False

            def handle_click(event: events.SceneClickEventArguments) -> None:
                if event.click_type == 'dblclick':
                    position = self.system.odometer.prediction.point
                    if self.scene_look:
                        self.scene_look = False
                        height = 10
                        x = position.x-0.5
                        y = position.y-0.5
                    else:
                        self.scene_look = True
                        height = 2
                        x = position.x + 0.8
                        y = position.y - 0.8
                    scene.move_camera(x=x, y=y, z=height,
                                      look_at_x=position.x, look_at_y=position.y)
                    return
            with ui.scene(650, 500, on_click=handle_click) as scene:
                robot_object(self.system.odometer, self.system.usb_camera_provider, self.system.field_friend)
                rosys.driving.driver_object(self.system.driver)
                plant_objects(self.system.plant_provider, self.system.big_weed_category_names +
                              self.system.small_weed_category_names)
                visualizer_object(self.system.automator, self.system.path_provider, self.system.mowing)
                field_object(self.system.field_provider)
                scene.move_camera(-0.5, -1, 2)
            with ui.row().classes('m-4'):
                key_controls = KeyControls(self.system)
                rosys.driving.joystick(self.system.steerer, size=50, color='#6E93D6').classes(
                    'm-4').style('width:12em; height:12em;')
                with ui.column().classes('mt-4'):
                    with ui.row():
                        ui.markdown(SHORTCUT_INFO).classes('col-grow')
                        ui.number('speed', format='%.0f', max=4, min=1, value=1).props('dense outlined').classes(
                            'w-24 mr-4').bind_value(key_controls, 'speed').tooltip('Set the speed of the robot (1-4)')

                        with ui.dialog() as dialog, ui.card():
                            ui.label('Do you want to continue the old mowing automation?')
                            with ui.row():
                                ui.button('Yes', on_click=lambda: dialog.submit('Yes'))
                                ui.button('No', on_click=lambda: dialog.submit('No'))
                                ui.button('Cancel', on_click=lambda: dialog.submit('Cancel'))

                        async def ensure_start() -> bool:
                            self.log.info('Ensuring start of automation')
                            if not automations_toggle.value == 'mowing' or self.system.mowing.current_path is None:
                                return True
                            result = await dialog
                            if result == 'Yes':
                                self.system.mowing.continue_mowing = True
                            elif result == 'No':
                                self.system.mowing.continue_mowing = False
                            elif result == 'Cancel':
                                return False
                            return True
                    with ui.row().classes('items-center'):
                        automation_controls(self.system.automator, can_start=ensure_start)
                        if not system.is_real:
                            ui.number('Roll', format='%.2f', value=0, step=1, min=-180, max=180).props(
                                    'dense outlined suffix=Deg').classes('w-24').bind_value(
                                    self.system.field_friend.imu, 'roll').tooltip(
                                    'Set teh roll for a simulated Robot')
                            ui.number('Pitch', format='%.2f', value=0, step=1, min=-180, max=180).props(
                                    'dense outlined suffix=Deg').classes('w-24').bind_value(
                                    self.system.field_friend.imu, 'pitch').tooltip(
                                    'Set teh tilt for a simulated Robot')
                            ui.button(text='rolled?',on_click=lambda :self.system.rolling.is_rolled(),)
                            ui.button(text='pitched?',on_click=lambda :self.system.rolling.is_pitched(),)

                        @ui.refreshable
                        def show_field_selection() -> None:
                            def set_field() -> None:
                                for field in self.system.field_provider.fields:
                                    if field.id == self.field_selection.value:
                                        self.field = field
                                        self.system.weeding.field = field
                                        self.system.weeding_new.field = field
                                        show_row_selection.refresh()

                            self.field_selection = ui.select(
                                [field.id for field in self.system.field_provider.fields],
                                with_input=True, on_change=set_field, label='Field').tooltip(
                                'Select the field to weed').classes('w-24')
                            show_row_selection()

                        @ui.refreshable
                        def show_row_selection() -> None:
                            def set_row() -> None:
                                for row in self.field.rows:
                                    if row.id == self.row_selection.value:
                                        self.system.weeding.row = row
                                        self.system.weeding_new.start_row = row
                            if self.field is not None:
                                self.row_selection = ui.select(
                                    [row.id for row in self.field.rows if self.field is not None],
                                    label='Row', with_input=True, on_change=set_row).tooltip(
                                    'Select the row to weed').classes('w-24')
                        show_field_selection()
                        self.system.field_provider.FIELDS_CHANGED.register(show_field_selection.refresh)

                    with ui.row():
                        automations_toggle = ui.toggle(
                            [key for key in self.system.automations.keys()],
                            value='weeding').bind_value(
                            self.system.automator, 'default_automation', forward=lambda key: self.system.automations[key],
                            backward=lambda automation: next(
                                key for key, value in self.system.automations.items() if value == automation))
                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            ui.number('padding', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'padding').tooltip('Set the padding for the mowing automation')
                            ui.number('lane distance', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.mowing, 'lane_distance').tooltip('Set the lane distance for the system. automation')
                            ui.number('number of outer lanes', value=3, step=1, min=3, format='%.0f').props('dense outlined').classes(
                                'w-24').bind_value(system.mowing, 'number_of_outer_lanes').tooltip('Set the number of outer lanes for the mowing automation')

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='demo_weeding'):
                        if system.field_friend.z_axis:
                            ui.number('Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props('dense outlined suffix=m').classes(
                                'w-24').bind_value(system.demo_weeding, 'drill_depth').tooltip('Set the drill depth for the weeding automation')
                            ui.label('press PLAY to start weeding with the set drill depth')
                        else:
                            ui.label('This Field Friend has no weeding tool available')

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='weeding'):
                        with ui.row():
                            mode = ui.toggle(
                                ['Bohren', 'Hacken'],
                                value='Bohren').bind_value(
                                system.demo_weeding, 'mode').props('outline')
                            ui.number(
                                'Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.weeding, 'drill_depth').tooltip(
                                'Set the drill depth for the weeding automation').bind_visibility_from(
                                mode, 'value', value='Bohren')

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='Continuous weeding'):
                        with ui.row():
                            ui.number(
                                'Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.weeding_new, 'drill_depth').tooltip(
                                'Set the drill depth for the weeding automation')

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='collecting'):
                        with ui.row():
                            ui.number(
                                'Drill angle', format='%.0f', value=100, step=1, min=1, max=180).props(
                                'dense outlined suffix=m').classes('w-24').bind_value(
                                self.system.coin_collecting, 'angle').tooltip(
                                'Set the drill depth for the weeding automation')
                            ui.checkbox('with drilling', value=True).bind_value(
                                self.system.coin_collecting, 'with_drilling')

            with ui.row().classes('m-4'):
                ui.button('emergency stop', on_click=lambda: system.field_friend.estop.set_soft_estop(True)).props('color=red').classes(
                    'py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop, 'is_soft_estop_active', value=False)
                ui.button('emergency reset', on_click=lambda: system.field_friend.estop.set_soft_estop(False)).props(
                    'color=red-700 outline').classes('py-3 px-6 text-lg').bind_visibility_from(system.field_friend.estop,
                                                                                               'is_soft_estop_active', value=True)
                ui.checkbox(
                    'Space bar emergency stop').tooltip(
                    'Enable or disable the emergency stop on space bar').bind_value(key_controls, 'estop_on_space')
