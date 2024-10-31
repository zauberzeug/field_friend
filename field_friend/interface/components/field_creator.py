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
        self.field_name: str = 'Field'
        self.row_spacing: float = 0.5
        self.row_count: int = 10
        self.outline_buffer_width: float = 2.0
        self.bed_count: int = 1
        self.bed_spacing: float = 0.5
        self.next: Callable = self.find_first_row

        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                self.row_sight = ui.interactive_image().classes('w-3/5')
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    # NOTE: the next function is replaced, hence we need the lambda
                    ui.button('Next', on_click=lambda: self.next())  # pylint: disable=unnecessary-lambda
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
            ui.label('1. Drive the robot to the leftmost row of your field.').classes(
                'text-lg')
            ui.label('2. Place the robot about 1 meter in front of the first crop.').classes(
                'text-lg')
            ui.label('• The blue line should be in the center of the row.') \
                .classes('text-lg ps-8')
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
            ui.input('Field Name') \
                .props('dense outlined').classes('w-40') \
                .tooltip('Enter a name for the field') \
                .bind_value(self, 'field_name')
            beds_switch = ui.switch('Field has multiple beds')
            ui.number('Number of Beds',
                      value=10, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the number of beds.')\
                .bind_value(self, 'bed_count').bind_visibility_from(beds_switch, 'value')
            ui.number('Bed Spacing', suffix='cm',
                      value=50, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the beds') \
                .bind_value(self, 'bed_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0) \
                .bind_visibility_from(beds_switch, 'value')
            ui.number('Number of Rows (per Bed)',
                      value=10, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the number of rows (per bed, if multiple beds are selected).')\
                .bind_value(self, 'row_count')
            ui.number('Row Spacing', suffix='cm',
                      value=50, step=1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the distance between the rows') \
                .bind_value(self, 'row_spacing', forward=lambda v: v / 100.0, backward=lambda v: v * 100.0)
            ui.number('Outline Buffer Width', suffix='m',
                      value=2, step=0.1, min=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Set the width of the buffer around the field outline') \
                .bind_value(self, 'outline_buffer_width')
        self.next = self.find_row_ending

    def find_row_ending(self) -> None:
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('1. Drive the robot to the end of the current row.') \
                .classes('text-lg')
            ui.label('2. Place the robot about 1 meter after the last crop.') \
                .classes('text-lg')
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
                ui.label(f'Field Name: {self.field_name}').classes('text-lg')
                ui.label(f'First Row Start: {self.first_row_start}').classes('text-lg')
                ui.label(f'First Row End: {self.first_row_end}').classes('text-lg')
                if self.bed_count > 1:
                    ui.label(f'Number of Beds: {self.bed_count}').classes('text-lg')
                    ui.label(f'Bed Spacing: {self.bed_spacing*100} cm').classes('text-lg')
                ui.label(f'Row Spacing: {self.row_spacing*100} cm').classes('text-lg')
                ui.label(f'Number of Rows (per Bed): {self.row_count}').classes('text-lg')
                ui.label(f'Outline Buffer Width: {self.outline_buffer_width} m').classes('text-lg')
            with ui.row().classes('items-center'):
                ui.button('Cancel', on_click=self.dialog.close).props('color=red')
        self.next = self._apply

    def _apply(self) -> None:
        self.dialog.close()
        if self.first_row_start is None or self.first_row_end is None:
            ui.notify('No valid field parameters.')
            return
        if self.bed_count > 1:
            self.field_provider.create_field(Field(id=str(uuid4()),
                                                   name=self.field_name,
                                                   first_row_start=self.first_row_start,
                                                   first_row_end=self.first_row_end,
                                                   row_spacing=self.row_spacing,
                                                   row_count=int(self.row_count),
                                                   outline_buffer_width=self.outline_buffer_width,
                                                   bed_count=int(self.bed_count),
                                                   bed_spacing=self.bed_spacing))
        else:
            self.field_provider.create_field(Field(id=str(uuid4()),
                                                   name=self.field_name,
                                                   first_row_start=self.first_row_start,
                                                   first_row_end=self.first_row_end,
                                                   row_spacing=self.row_spacing,
                                                   row_count=int(self.row_count),
                                                   outline_buffer_width=self.outline_buffer_width))
        self.first_row_start = None
        self.first_row_end = None

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
