
import logging
from typing import TYPE_CHECKING, Callable

from nicegui import events, ui
from rosys.automation import Automator
from rosys.driving import Driver, Odometer, Steerer, driver_object, joystick
from rosys.vision import CameraProvider

from ..automations import FieldProvider, Mowing, PathProvider, PlantDetector, PlantProvider, Puncher, Weeding
from ..hardware import FieldFriend
from .automation_controls import automation_controls
from .field_object import field_object
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object
from .visualizer_object import visualizer_object

if TYPE_CHECKING:
    from system import System

SHORTCUT_INFO = '''
    Steer the robot manually with the JOYSTICK on the left. <br>
    Or hold SHIFT and use the ARROW KEYS
'''


class operation:

    def __init__(
        self,
        field_friend: FieldFriend,
        steerer: Steerer,
        driver: Driver,
        automator: Automator,
        odometer: Odometer,
        camera_provider: CameraProvider,
        plant_provider: PlantProvider,
        plant_detector: PlantDetector,
        puncher: Puncher,
        weeding: Weeding,
        mowing: Mowing,
        path_provider: PathProvider,
        field_provider: FieldProvider,
        automations: dict[str, Callable],
        system: 'System',
    ) -> None:
        self.field_provider = field_provider
        self.mowing = mowing
        self.log = logging.getLogger('field_friend.operation')

        with ui.card().tight():
            self.scene_look = False

            def handle_click(event: events.SceneClickEventArguments) -> None:
                if event.click_type == 'dblclick':
                    position = odometer.prediction.point
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
                robot_object(odometer, camera_provider, field_friend)
                driver_object(driver)
                plant_objects(plant_provider, plant_detector.weed_category_names)
                visualizer_object(automator, path_provider, mowing)
                field_object(field_provider)
                scene.move_camera(-0.5, -1, 2)
            with ui.row():
                key_controls = KeyControls(field_friend, steerer, automator, puncher, system)
                joystick(steerer, size=50, color='#6E93D6').classes('m-4').style('width:12em; height:12em;')
                with ui.column().classes('mt-4'):
                    with ui.row():
                        ui.markdown(SHORTCUT_INFO).classes('col-grow')
                        ui.number('speed', format='%.0f', max=4, min=1).props('dense outlined').classes(
                            'w-24 mr-4').bind_value(key_controls, 'speed').tooltip('Set the speed of the robot (1-4)')
                    with ui.row():
                        automations_toggle = ui.toggle(
                            [key for key in automations.keys()],
                            value='weeding').bind_value(
                            automator, 'default_automation', forward=lambda key: automations[key],
                            backward=lambda automation: next(
                                key for key, value in automations.items() if value == automation))
                        with ui.dialog() as dialog, ui.card():
                            ui.label('Do you want to continue the old mowing automation?')
                            with ui.row():
                                ui.button('Yes', on_click=lambda: dialog.submit('Yes'))
                                ui.button('No', on_click=lambda: dialog.submit('No'))
                                ui.button('Cancel', on_click=lambda: dialog.submit('Cancel'))

                        async def ensure_start() -> bool:
                            self.log.info('Ensuring start of automation')
                            if not automations_toggle.value == 'mowing' or mowing.current_path == None:
                                return True
                            result = await dialog
                            if result == 'Yes':
                                self.mowing.continue_mowing = True
                            elif result == 'No':
                                self.mowing.continue_mowing = False
                            elif result == 'Cancel':
                                return False
                            return True
                        automation_controls(automator, can_start=ensure_start)

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='mowing'):
                        with ui.row():
                            @ui.refreshable
                            def show_field_selection() -> None:

                                def set_field() -> None:
                                    for field in self.field_provider.fields:
                                        if field.id == self.field_selection.value:
                                            self.mowing.field = field

                                self.field_selection = ui.select(
                                    [field.id for field in self.field_provider.fields],
                                    with_input=True, on_change=set_field).tooltip('Select the field to mow')
                            show_field_selection()
                            self.field_provider.FIELDS_CHANGED.register(show_field_selection.refresh)
                            ui.number('padding', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(mowing, 'padding').tooltip('Set the padding for the mowing automation')
                            ui.number('lane distance', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value(mowing, 'lane_distance').tooltip('Set the lane distance for the mowing automation')
                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='weeding'):
                        if field_friend.z_axis is not None:
                            ui.number('Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props('dense outlined suffix=m').classes(
                                'w-24').bind_value(weeding, 'drill_depth').tooltip('Set the drill depth for the weeding automation')
                            ui.label('press PLAY to start weeding with the set drill depth')
                        else:
                            ui.label('This Field Friend has no weeding tool available')
            with ui.row().classes('m-4'):
                ui.button('emergency stop', on_click=lambda: field_friend.estop.set_soft_estop(True)).props('color=red').classes(
                    'py-3 px-6 text-lg').bind_visibility_from(field_friend.estop, 'is_soft_estop_active', value=False)
                ui.button('emergency reset', on_click=lambda: field_friend.estop.set_soft_estop(False)).props(
                    'color=red-700 outline').classes('py-3 px-6 text-lg').bind_visibility_from(field_friend.estop,
                                                                                               'is_soft_estop_active', value=True)
                ui.checkbox(
                    'Space bar emergency stop').tooltip(
                    'Enable or disable the emergency stop on space bar').bind_value(key_controls, 'estop_on_space')
