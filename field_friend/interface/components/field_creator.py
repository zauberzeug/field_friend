from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from field_friend.automations.navigation import StraightLineNavigation
from field_friend.interface.components.monitoring import CameraPosition

if TYPE_CHECKING:
    from field_friend.system import System


class FieldCreator:

    def __init__(self, system: 'System'):
        drive_straight = StraightLineNavigation(system, system.monitoring)
        drive_straight.length = 100  # NOTE: for now, every 100 m the user needs to re-start automation
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.automator = rosys.automation.Automator(system.steerer,
                                                    default_automation=drive_straight.start,
                                                    on_interrupt=system.field_friend.stop)
        self.steerer = system.steerer
        self.plant_locator = system.plant_locator
        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                self.row_sight = ui.interactive_image().classes('w-3/5')
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    self.next_button = ui.button('Ready')
        ui.timer(0.1, self.update_front_cam)
        self.open()

    def open(self):
        self.find_first_row()
        self.dialog.open()

    def find_first_row(self):
        self.headline.text = 'Drive to First Row'
        self.row_sight.content = '<line x1="50%" y1="0" x2="50%" y2="100%" stroke="#6E93D6" stroke-width="6"/>'
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Drive the robot to the leftmost row of your new field, '
                     'right before the first crop. '
                     'So that the blue line is in the center of the row.') \
                .classes('text-lg text-center')
        self.next_button.on_click(self.get_infos)
        self.next_button.text = 'Next'

    def get_infos(self):
        self.headline.text = 'Plant Information'
        self.row_sight.content = ''
        self.content.clear()
        crops = self.plant_locator.crop_category_names
        crops.remove('coin_with_hole')
        with self.content:
            ui.select(label='Cultivated Crop', options=crops).classes('w-40')
            ui.number('Crop Spacing', suffix='cm',
                      value=20, step=1, min=1, max=60) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the crops')
            ui.number('Row Spacing', suffix='cm',
                      value=50, step=5, min=20, max=100) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the rows')
        self.next_button.on_click(self.find_row_ending)

    def find_row_ending(self):
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                rosys.automation.automation_controls(self.automator)
            ui.label('Press "Play" to start driving forward. '
                     'At the end of the row, press the "Stop" button.') \
                .classes('text-lg text-center')
        self.next_button.on_click(self.drive_to_last_row)

    def drive_to_last_row(self):
        self.headline.text = 'Drive to Last Row'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Please drive the robot to the last row on this side, '
                     'right before the first crop.') \
                .classes('w-64 text-lg')
        self.next_button.on_click(self.confirm_geometry)

    def confirm_geometry(self):
        self.headline.text = 'Confirm Geometry'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                rosys.automation.automation_controls(self.automator)
            ui.label('Press "Play" to start driving forward. '
                     'At the end of the row, press the "Stop" button.') \
                .classes('w-64 text-lg')
        self.next_button.text = 'Create Field'
        self.next_button.on_click(self._apply)

    def _apply(self):
        self.dialog.close()
        # self.system.field_provider.create_field()

    def update_front_cam(self):
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
