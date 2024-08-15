from typing import TYPE_CHECKING, Callable
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
        drive_straight.linear_speed_limit = 2
        drive_straight.angular_speed_limit = 1
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
        self.row_spacing = 0.5
        self.padding = 1
        self.padding_bottom = 2
        self.next: Callable = self.find_first_row

        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                self.row_sight = ui.interactive_image().classes('w-3/5')
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    # NOTE: the next function is replaced, hence we need the lambda
                    ui.button('Next', on_click=lambda: self.next())
        ui.timer(0.1, self.update_front_cam)
        self.open()

    def open(self) -> None:
        self.next()
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
        self.next = self.get_infos

    def get_infos(self) -> None:
        assert self.gnss.current is not None
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_start = self.gnss.current.location

        self.row_sight.content = ''
        crops = self.plant_locator.crop_category_names[:]
        crops.remove('coin_with_hole')
        self.content.clear()
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
                .bind_value(self, 'row_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0)
        self.next = self.find_row_ending

    def find_row_ending(self) -> None:
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                rosys.automation.automation_controls(self.automator)
            ui.label('Press "Play" to start driving forward. '
                     'At the end of the row, press the "Stop" button.') \
                .classes('text-lg text-center')
        self.next = self.drive_to_last_row

    def drive_to_last_row(self) -> None:
        assert self.gnss.current is not None
        if not("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_end = self.gnss.current.location

        self.headline.text = 'Drive to Last Row'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Drive the robot to the last row on this side, '
                     'right before the first crop.') \
                .classes('text-lg text-center')
        self.next = self.confirm_geometry

    def confirm_geometry(self) -> None:
        assert self.gnss.current is not None
        if not("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.last_row_end = self.gnss.current.location

        assert self.first_row_end is not None
        assert self.last_row_end is not None
        if not self.build_geometry():
            d = self.first_row_end.distance(self.last_row_end)
            ui.label(f'The distance between first row and last row is {d:.2f} m. '
                     f'Which does not match well with the provided row spacing of {self.row_spacing} cm.') \
                .classes('text-red')

        self.headline.text = 'Confirm Geometry'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                rosys.automation.automation_controls(self.automator)
            ui.label('Press "Play" to start driving forward. '
                     'At the end of the row, press the "Stop" button.') \
                .classes('w-64 text-lg')
        self.next = self._apply

    def build_geometry(self) -> bool:
        """Build the geometry of the field based on the given points.

        Returns True if the row spacing matches the distance between the first and last row, False otherwise.
        Will create rows in any case to make testing easier.
        """
        assert self.first_row_start is not None
        assert self.first_row_end is not None
        assert self.last_row_end is not None
        distance = self.first_row_end.distance(self.last_row_end)
        number_of_rows = distance / (self.row_spacing) + 1
        # get AB line
        a = self.first_row_start.cartesian()
        b = self.first_row_end.cartesian()
        c = self.last_row_end.cartesian()
        ab = a.direction(b)
        bc = b.direction(c)
        d = a.polar(distance, bc)
        for i in range(int(number_of_rows)):
            start = a.polar(i * self.row_spacing, bc)
            end = b.polar(i * self.row_spacing, bc)
            self.field.rows.append(Row(id=str(uuid4()), name=f'Row #{len(self.field.rows)}',
                                       points=[self.first_row_start.shifted(start),
                                               self.first_row_start.shifted(end)]
                                       ))
        bottom_left = a.polar(-self.padding_bottom, ab).polar(-self.padding, bc)
        top_left = b.polar(self.padding, ab).polar(-self.padding, bc)
        top_right = c.polar(self.padding, ab).polar(self.padding, bc)
        bottom_right = d.polar(-self.padding_bottom, ab).polar(self.padding, bc)
        self.field.points = [self.first_row_start.shifted(p) for p in [bottom_left, top_left, top_right, bottom_right]]
        return 1 - number_of_rows % 1 < 0.1

    def _apply(self) -> None:
        self.dialog.close()
        self.field_provider.fields.append(self.field)
        self.field_provider.request_backup()
        self.field_provider.FIELDS_CHANGED.emit()

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
