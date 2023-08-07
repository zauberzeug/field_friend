
from typing import Callable

from nicegui import events, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, Steerer, driver_object, joystick
from rosys.vision import CameraProvider

from ..automations import Mowing, Puncher, Weeding, plant_detector, plant_provider
from ..hardware import FieldFriend
from ..navigation import FieldProvider, PathProvider
from .field_object import field_object
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object
from .visualizer_object import visualizer_object

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
        plant_provider: plant_provider,
        plant_detector: plant_detector,
        puncher: Puncher,
        weeding: Weeding,
        mowing: Mowing,
        path_provider: PathProvider,
        field_provider: FieldProvider,
        automations: dict[str, Callable],
    ) -> None:
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
                # if event.click_type == 'click':
                #     scene.move_camera(x=event.hits[0].x-0.5, y=event.hits[0].y-0.5, z=2,
                #                       look_at_x=event.hits[0].x, look_at_y=event.hits[0].y)
            with ui.scene(650, 500, on_click=handle_click) as scene:
                robot_object(odometer, camera_provider, field_friend)
                driver_object(driver)
                plant_objects(plant_provider, plant_detector.weed_category_names)
                visualizer_object(automator, path_provider, mowing)
                field_object(field_provider)
                scene.move_camera(-0.5, -1, 2)
            with ui.row():
                key_controls = KeyControls(field_friend, steerer, automator, puncher)
                joystick(steerer, size=50, color='#6E93D6').classes('m-4').style('width:12em; height:12em;')
                with ui.column().classes('mt-4'):
                    with ui.row():
                        ui.markdown(SHORTCUT_INFO).classes('col-grow')
                        ui.number('speed', format='%.0f', max=4, min=1).props('dense outlined').classes(
                            'w-24 mr-4').bind_value(key_controls, 'speed').tooltip('Set the speed of the robot (1-4)')
                    with ui.row():
                        def change_default_automation() -> None:
                            automator.default_automation = automations[automations_toggle.value]
                        automations_toggle = ui.toggle(
                            [key for key in automations.keys()],
                            value='weeding', on_change=change_default_automation)
                        automation_controls(automator)

                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='mowing'):
                        with ui.row():

                            def set_field() -> None:
                                for field in field_provider.fields:
                                    if field.name == field_selection.value:
                                        mowing.field = field

                            def change_field_selections() -> None:
                                field_selection.options = [field.name for field in field_provider.fields]
                                field_selection.value = mowing.field.name if mowing.field is not None else None

                            field_selection = ui.select(
                                [field.name for field in field_provider.fields],
                                with_input=True, on_change=set_field).tooltip('Select the field to mow')
                            field_provider.FIELDS_CHANGED.register(change_field_selections)

                            ui.number('padding', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value_to(mowing, 'padding').tooltip('Set the padding for the mowing automation')
                            ui.number('lane distance', value=0.5, step=0.1, min=0.0, format='%.1f').props('dense outlined suffix=m').classes(
                                'w-24').bind_value_to(mowing, 'lane_distance').tooltip('Set the lane distance for the mowing automation')
                    with ui.column().bind_visibility_from(automations_toggle, 'value', value='weeding'):
                        if field_friend.z_axis is not None:
                            ui.number('Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.18).props('dense outlined suffix=cm').classes(
                                'w-24').bind_value(weeding, 'drill_depth').tooltip('Set the drill depth for the weeding automation')
                            ui.label('press PLAY to start weeding with the set drill depth')
                        else:
                            ui.label('This Field Friend has no weeding tool available')
            with ui.row().classes('m-4'):
                ui.button('emergency stop', on_click=field_friend.estop.software_emergency_stop).props('color=red').classes(
                    'py-3 px-6 text-lg').bind_visibility_from(field_friend.estop, 'en3_active', value=False)
                ui.button('emergency reset', on_click=field_friend.estop.release_en3).props(
                    'color=red-700 outline').classes('py-3 px-6 text-lg').bind_visibility_from(field_friend.estop, 'en3_active')
                ui.checkbox(
                    'Space bar emergency stop').tooltip(
                    'Enable or disable the emergency stop on space bar').bind_value(key_controls, 'estop_on_space')
