from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from field_friend.automations.navigation import StraightLineNavigation

if TYPE_CHECKING:
    from field_friend.system import System


class FieldCreator:

    def __init__(self, system: 'System'):
        drive_straight = StraightLineNavigation(system, system.monitoring)
        drive_straight.length = 100  # NOTE: for now, every 100 m the user needs to re-start automation
        automator = rosys.automation.Automator(system.steerer,
                                               default_automation=drive_straight.start,
                                               on_interrupt=system.field_friend.stop)
        with ui.dialog() as self.dialog:
            with ui.stepper().props('vertical').classes('w-full') as stepper:
                with ui.step('Drive to First Row'):
                    with ui.row().classes('items-center'):
                        rosys.driving.joystick(system.steerer, size=50, color='#6E93D6')
                        ui.label('Please drive the robot to the leftmost row of your new field, '
                                 'right before the first crop. '
                                 'So that the blue line runs over the row.') \
                            .classes('w-64 text-lg')
                    with ui.stepper_navigation():
                        ui.button('Ready', on_click=stepper.next)
                with ui.step('Plant Information'):
                    ui.select(label='Cultivated Crop', options=system.plant_locator.crop_category_names).classes('w-40')
                    ui.number('Crop Spacing', suffix='cm',
                              value=20, step=1, min=1, max=60) \
                        .props('dense outlined').classes('w-40') \
                        .tooltip('Set the distance between the crops')
                    ui.number('Row Spacing', suffix='cm',
                              value=50, step=5, min=20, max=100) \
                        .props('dense outlined').classes('w-40') \
                        .tooltip('Set the distance between the rows')
                    with ui.stepper_navigation():
                        ui.button('Ready', on_click=stepper.next)
                        ui.button('Back', on_click=stepper.previous).props('flat')
                with ui.step('Find Row Ending'):
                    with ui.row().classes('items-center'):
                        rosys.automation.automation_controls(automator)
                        ui.label('Press "Play" to start driving forward. '
                                 'At the end of the row, press the "Stop" button.') \
                            .classes('w-64 text-lg')
                    with ui.stepper_navigation():
                        ui.button('Ready', on_click=stepper.next)
                        ui.button('Back', on_click=stepper.previous).props('flat')
                with ui.step('Drive to Last Row'):
                    with ui.row().classes('items-center'):
                        rosys.driving.joystick(system.steerer, size=50, color='#6E93D6')
                        ui.label('Please drive the robot to the last row on this side, '
                                 'right before the first crop. '
                                 'So that the blue line runs over the row.') \
                            .classes('w-64 text-lg')
                    with ui.stepper_navigation():
                        ui.button('Ready', on_click=stepper.next)
                        ui.button('Back', on_click=stepper.previous).props('flat')
                with ui.step('Confirm Geometry'):
                    with ui.row().classes('items-center'):
                        rosys.automation.automation_controls(automator)
                        ui.label('Press "Play" to start driving forward. '
                                 'At the end of the row, press the "Stop" button.') \
                            .classes('w-64 text-lg')
                    with ui.stepper_navigation():
                        ui.button('Create Field', on_click=self._apply)
                        ui.button('Back', on_click=stepper.previous).props('flat')
        stepper.value = 'Drive to Last Row'
        self.dialog.open()

    def _apply(self):
        self.dialog.close()
        # self.system.field_provider.create_field()
