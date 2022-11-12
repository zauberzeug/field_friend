import rosys
from nicegui import ui

import interface


def operation(steerer: rosys.driving.Steerer, automator: rosys.automation.Automator, odometer: rosys.driving.Odometer) -> None:
    with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
        # steering
        with ui.card().tight():
            with ui.row():
                rosys.driving.keyboard_control(steerer)
                rosys.driving.joystick(steerer, size=50, color='#6E93D6')

                with ui.column().classes('mt-2').style('width:34em;gap:0.75em'):
                    ui.markdown('manual steer the robot with the JOYSTICK on the left or <br>press SHIFT and use the arrow keys on your keyboard')\
                        .classes('col-grow')
                    with ui.row():
                        rosys.automation.automation_controls(automator)
                    ui.label('press PLAY to start weeding')
            with ui.scene(640, 480).style('margin-top:-0.5em') as scene:
                interface.robot(odometer)
