import rosys
from nicegui import ui

import hardware
import interface


def operation(
    robot: hardware.Robot,
    steerer: rosys.driving.Steerer,
    automator: rosys.automation.Automator,
    odometer: rosys.driving.Odometer,
    camera_provider: rosys.vision.CameraProvider,
) -> None:
    with ui.card().tight():
        with ui.scene(640, 460) as scene:
            interface.robot_object(odometer, camera_provider, robot)
            scene.move_camera(-0.5, -1, 1.3)
        with ui.row():
            key_controls = interface.KeyControls(robot, steerer, automator)
            rosys.driving.joystick(steerer, size=50, color='#6E93D6').classes(
                'm-4').style('width:15em; height:15em;')
            with ui.column().classes('mt-4'):
                ui.markdown(
                    'Steer the robot manually with the JOYSTICK on the left. <br>Hold SHIFT and use the ARROW KEYS to steer the robot \
                        <br>or press ! to HOME both axis and move them with WASD').classes('col-grow')
                with ui.row():
                    def check_depth(depth: float) -> None:
                        if depth > robot.MAX_Z*100:
                            depth_number.set_value(robot.MAX_Z*100)
                        if depth < robot.MIN_Z*100:
                            depth_number.set_value(robot.MIN_Z*100)
                    speed_number = ui.number('Robot speed').props(
                        'dense outlined').classes('w-24').bind_value(key_controls, 'speed')
                    depth_number = ui.number('Drill depth', format='%.2f', on_change=lambda e: check_depth(e.value)).props(
                        'dense outlined suffix=cm').classes('w-28').bind_value(robot, 'zaxis_drill_depth',
                                                                               backward=lambda x: x * 100, forward=lambda x: x / 100)
                with ui.row():
                    def stop():
                        if automator.is_running:
                            automator.stop(because='emergency stop triggered')
                    rosys.automation.automation_controls(automator)

                    robot.ESTOP_TRIGGERED.register(stop)
                ui.label('press PLAY to start weeding with the set drill depth')
