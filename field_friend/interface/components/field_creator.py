from typing import TYPE_CHECKING
from uuid import uuid4

import rosys
from nicegui import ui

from field_friend.automations import Field, Row
from field_friend.automations.navigation import StraightLineNavigation
from field_friend.interface.components.monitoring import CameraPosition
from field_friend.localization import GeoPoint

if TYPE_CHECKING:
    from field_friend.system import System


class FieldCreator:

    def __init__(self, system: 'System'):
        drive_straight = StraightLineNavigation(system, system.monitoring)
        drive_straight.length = 100  # NOTE: for now, every 100 m the user needs to re-start automation
        drive_straight.linear_speed_limit = 0.6
        drive_straight.angular_speed_limit = 0.2
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.automator = rosys.automation.Automator(system.steerer,
                                                    default_automation=drive_straight.start,
                                                    on_interrupt=system.field_friend.stop)
        self.steerer = system.steerer
        self.plant_locator = system.plant_locator
        self.gnss = system.gnss
        self.field_provider = system.field_provider

        self.field = Field(id=str(uuid4()), name=f'Field #{len(self.field_provider.fields) + 1}')
        self.first_row_start: GeoPoint | None = None
        self.first_row_end: GeoPoint | None = None
        self.last_row_end: GeoPoint | None = None
        self.row_spacing = 50

        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                self.row_sight = ui.interactive_image().classes('w-3/5')
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    self.next_button = ui.button('Ready')
        ui.timer(0.1, self.update_front_cam)
        self.open()

    def open(self) -> None:
        self.find_first_row()
        self.dialog.open()

    def find_first_row(self) -> None:
        self.headline.text = 'Drive to First Row'
        self.row_sight.content = '<line x1="50%" y1="0" x2="50%" y2="100%" stroke="#6E93D6" stroke-width="6"/>'
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Drive the robot to the leftmost row of your new field, '
                     'right before the first crop. '
                     'The blue line should be in the center of the row.') \
                .classes('text-lg text-center')
        self.next_button.on_click(self.get_infos)
        self.next_button.text = 'Next'

    def get_infos(self) -> None:
        assert self.gnss.current is not None
        if self.gnss.current.gps_qual == 4:
            self.first_row_start = self.gnss.current.location
        else:
            ui.label('No RTK fix available.').classes('text-red')

        self.row_sight.content = ''
        self.content.clear()
        crops = self.plant_locator.crop_category_names[:]
        crops.remove('coin_with_hole')
        with self.content:
            ui.select(label='Cultivated Crop', options=crops, clearable=True).classes('w-40') \
                .bind_value(self.field, 'crop')
            ui.number('Crop Spacing', suffix='cm',
                      value=20, step=1, min=1, max=60) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the crops')
            ui.number('Row Spacing', suffix='cm',
                      value=50, step=5, min=20, max=100) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the rows') \
                .bind_value(self, 'row_spacing')
        self.next_button.on_click(self.find_row_ending)

    def find_row_ending(self) -> None:
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                rosys.automation.automation_controls(self.automator)
            ui.label('Press "Play" to start driving forward. '
                     'At the end of the row, press the "Stop" button.') \
                .classes('text-lg text-center')
        self.next_button.on_click(self.drive_to_last_row)

    def drive_to_last_row(self) -> None:
        assert self.gnss.current is not None
        if self.gnss.current.gps_qual == 4:
            self.first_row_end = self.gnss.current.location
        else:
            ui.label('No RTK fix available.').classes('text-red')

        self.headline.text = 'Drive to Last Row'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Drive the robot to the last row on this side, '
                     'right before the first crop.') \
                .classes('text-lg text-center')
        self.next_button.on_click(self.confirm_geometry)

    def confirm_geometry(self) -> None:
        assert self.gnss.current is not None
        if self.gnss.current.gps_qual == 4:
            self.last_row_end = self.gnss.current.location
        else:
            ui.label('No RTK fix available.').classes('text-red')

        if not self.build_geometry():
            ui.label('The geometry could not be created.').classes('text-red')
            return
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

    def build_geometry(self) -> bool:
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        assert self.last_row_end is not None
        self.field.reference = self.first_row_start
        distance = self.first_row_end.distance(self.last_row_end)
        number_of_rows = distance / (self.row_spacing / 100.0) + 1
        if 1 - number_of_rows % 1 > 0.1:
            ui.notification(f'The row spacing of {self.row_spacing} might be incorrect. Please check the distance between the rows.',
                            type='warning')
            return False
        # get AB line
        a = self.first_row_start.cartesian(self.field.reference)
        b = self.first_row_end.cartesian(self.field.reference)
        c = self.last_row_end.cartesian(self.field.reference)
        bc = b.direction(c)
        for i in range(int(number_of_rows)):
            start = a.polar(i * self.row_spacing, bc)
            end = b.polar(i * self.row_spacing, bc)
            self.field.rows.append(Row(id=str(uuid4()), name=f'Row #{len(self.field.rows)}',
                                       points=[self.field.reference.shifted(start),
                                               self.field.reference.shifted(end)]
                                       ))
        return True

    def _apply(self) -> None:
        self.dialog.close()
        # self.system.field_provider.create_field()

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
