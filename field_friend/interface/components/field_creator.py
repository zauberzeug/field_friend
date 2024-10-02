from typing import TYPE_CHECKING, Callable
from uuid import uuid4

import rosys
from nicegui import ui

from field_friend.automations.field import Field
from field_friend.interface.components.monitoring import CameraPosition
from field_friend.localization import GeoPoint

if TYPE_CHECKING:
    from field_friend.system import System


class FieldCreator:

    def __init__(self, system: 'System'):
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.steerer = system.steerer
        self.gnss = system.gnss
        self.field_provider = system.field_provider
        self.first_row_start: GeoPoint | None = None
        self.first_row_end: GeoPoint | None = None
        self.row_spacing = 0.5
        self.row_number = 10
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
            ui.label('Place the back center of the robot over the start point of the row.') \
                .classes('text-lg text-center')
        self.next = self.get_infos

    def get_infos(self) -> None:
        self.headline.text = 'Field Parameters'
        assert self.gnss.current is not None
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_start = self.gnss.current.location
        self.row_sight.content = ''
        self.content.clear()
        with self.content:
            ui.number('Number of rows',
                      value=10, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the number of rows.')\
                .bind_value(self, 'row_number')
            ui.number('Row Spacing', suffix='cm',
                      value=50, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the rows') \
                .bind_value(self, 'row_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0)
        self.next = self.find_row_ending

    def find_row_ending(self) -> None:
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('Drive the robot to the end of the current row.') \
                .classes('text-lg text-center')
            ui.label('Place the back center of the robot over the end point of the row.') \
                .classes('text-lg text-center')
        self.next = self.confirm_geometry

    def confirm_geometry(self) -> None:
        assert self.gnss.current is not None
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_end = self.gnss.current.location
        assert self.first_row_end is not None
        self.headline.text = 'Confirm Geometry'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                ui.label(f'First Row Start: {self.first_row_start}').classes('text-lg')
                ui.label(f'First Row End: {self.first_row_end}').classes('text-lg')
                ui.label(f'Row Spacing: {self.row_spacing} m').classes('text-lg')
                ui.label(f'Number of Rows: {self.row_number}').classes('text-lg')
            with ui.row().classes('items-center'):
                ui.button('Cancel', on_click=self.dialog.close).props('color=red')
        self.next = self._apply

    def _apply(self) -> None:
        self.dialog.close()
        if self.first_row_start is None or self.first_row_end is None:
            ui.notify('No valid field parameters.')
            return
        self.field_provider.create_field(Field(id=str(uuid4()),
                                               name='Field 1',
                                               first_row_start=self.first_row_start,
                                               first_row_end=self.first_row_end,
                                               row_spacing=self.row_spacing,
                                               row_number=self.row_number))
        self.first_row_start = None
        self.first_row_end = None

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
