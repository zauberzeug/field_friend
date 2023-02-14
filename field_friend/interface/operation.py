import rosys
from nicegui import ui
from rosys.automation import Automator, automation_controls
from rosys.driving import Odometer, Steerer, joystick
from rosys.hardware import Wheels
from rosys.vision import CameraProvider

from ..automations import Puncher, plant_provider
from ..hardware import EStop, FieldFriend, YAxis, ZAxis
from ..old_hardware import Robot
from .key_controls import KeyControls
from .plant_object import plant_objects
from .robot_object import robot_object


def operation(
    field_friend: FieldFriend,
    steerer: Steerer,
    automator: Automator,
    odometer: Odometer,
    camera_provider: CameraProvider,
    plant_provider: plant_provider,
    puncher: Puncher
) -> None:
    with ui.card().tight():
        with ui.scene(640, 460) as scene:
            robot_object(odometer, camera_provider, field_friend.y_axis, field_friend.z_axis)
            plant_objects(plant_provider)
            scene.move_camera(-0.5, -1, 1.3)
        with ui.row():
            key_controls = KeyControls(field_friend, steerer, automator, puncher)
            joystick(steerer, size=50, color='#6E93D6').classes(
                'm-4').style('width:15em; height:15em;')
            with ui.column().classes('mt-4'):
                ui.markdown(
                    'Steer the robot manually with the JOYSTICK on the left. <br>Hold SHIFT and use the ARROW KEYS to steer the robot \
                        <br>or press ! to HOME both axis and move them with WASD').classes('col-grow')
                with ui.row():
                    def check_depth(depth: float) -> None:
                        if depth > field_friend.z_axis.MAX_Z*100:
                            depth_number.set_value(field_friend.z_axis.MAX_Z*100)
                        if depth < field_friend.z_axis.MIN_Z*100:
                            depth_number.set_value(field_friend.z_axis.MIN_Z*100)
                    speed_number = ui.number('Robot speed').props(
                        'dense outlined').classes('w-24').bind_value(key_controls, 'speed')
                    depth_number = ui.number('Drill depth', format='%.2f', on_change=lambda e: check_depth(e.value)).props(
                        'dense outlined suffix=cm').classes('w-28').bind_value(field_friend.z_axis, 'zaxis_drill_depth',
                                                                               backward=lambda x: x * 100, forward=lambda x: x / 100)
                with ui.row():
                    def stop():
                        if automator.is_running:
                            automator.stop(because='emergency stop triggered')
                    automation_controls(automator)

                    field_friend.e_stop.ESTOP_TRIGGERED.register(stop)
                ui.label('press PLAY to start weeding with the set drill depth')
