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
            interface.KeyControls(robot, steerer, automator)
            rosys.driving.joystick(steerer, size=50, color='#6E93D6').classes('m-4')
            with ui.column().classes('mt-4'):
                ui.markdown('steer the robot manually with the JOYSTICK on the left or <br>hold SHIFT and use the ARROW KEYS on your keyboard')\
                    .classes('col-grow')
                with ui.row():
                    def stop():
                        if automator.is_running:
                            automator.stop(because='emergency stop triggered')
                    rosys.automation.automation_controls(automator)
                    robot.ESTOP_TRIGGERED.register(stop)
                ui.label('press PLAY to start weeding')
