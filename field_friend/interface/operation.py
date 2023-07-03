
from nicegui import ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Odometer, Steerer, joystick
from rosys.vision import CameraProvider

from ..automations import Puncher, Weeding, plant_detector, plant_provider
from ..hardware import FieldFriend
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object

SHORTCUT_INFO = '''
    Steer the robot manually with the JOYSTICK on the left. <br>
    Or hold SHIFT and use the ARROW KEYS
'''


def operation(
    field_friend: FieldFriend,
    steerer: Steerer,
    automator: Automator,
    odometer: Odometer,
    camera_provider: CameraProvider,
    plant_provider: plant_provider,
    plant_detector: plant_detector,
    puncher: Puncher,
    weeding: Weeding,
) -> None:
    with ui.card().tight():
        with ui.scene(720, 540) as scene:
            robot_object(odometer, camera_provider, field_friend)
            plant_objects(plant_provider, plant_detector.weed_category_names)
            scene.move_camera(-0.5, -1, 1.3)
        with ui.row():
            key_controls = KeyControls(field_friend, steerer, automator, puncher)
            joystick(steerer, size=50, color='#6E93D6').classes(
                'm-4').style('width:15em; height:15em;')
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
