
from typing import Optional

from nicegui import events, ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Driver, Odometer, Steerer, driver_object, joystick, keyboard_control
from rosys.pathplanning import path_object
from rosys.vision import CameraProvider

from ..automations import PathRecorder, Puncher, Weeding, plant_detector, plant_provider
from ..hardware import FieldFriend
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object

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
        *, dev: bool = False,
        path_recorder: Optional[PathRecorder] = None,
    ) -> None:
        with ui.card().tight():
            def handle_click(event: events.SceneClickEventArguments) -> None:
                if event.click_type == 'dblclick':
                    position = odometer.prediction.point
                    scene.move_camera(x=position.x-0.5, y=position.y-1, z=1.5,
                                      look_at_x=position.x, look_at_y=position.y)
                    return
                if event.click_type == 'click':
                    scene.move_camera(x=event.hits[0].x-0.5, y=event.hits[0].y-1, z=1.5,
                                      look_at_x=event.hits[0].x, look_at_y=event.hits[0].y)
            with ui.scene(720, 540, on_click=handle_click) as scene:
                robot_object(odometer, camera_provider, field_friend)
                driver_object(driver)
                plant_objects(plant_provider, plant_detector.weed_category_names)
                self.path3d = path_object()
                scene.move_camera(-0.5, -1, 1.5)
            with ui.row():
                if dev:
                    key_controls = KeyControls(field_friend, steerer, automator, puncher)
                else:
                    key_controls = keyboard_control(steerer)
                joystick(steerer, size=50, color='#6E93D6').classes('m-4').style('width:15em; height:15em;')
                with ui.column().classes('mt-4'):
                    with ui.row():
                        ui.markdown(SHORTCUT_INFO).classes('col-grow')
                        ui.number('speed', format='%.0f', max=9, min=1).props(
                            'dense outlined').classes('w-24').bind_value(key_controls, 'speed')
                    with ui.row():
                        automation_controls(automator)
                        if field_friend.z_axis is not None:
                            ui.number('Drill depth', format='%.2f', value=0.05, step=0.01, min=0.01, max=0.20).props(
                                'dense outlined suffix=cm').classes('w-24').bind_value(weeding, 'drill_depth')

                    ui.label('press PLAY to start weeding with the set drill depth')
        if path_recorder is not None:
            path_recorder.PATH_DRIVING_STARTED.register(self.path3d.update)
