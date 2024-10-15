from typing import TYPE_CHECKING, Callable
from uuid import uuid4

import rosys
from nicegui import ui

from field_friend.automations.field import Field, RowSupportPoint
from field_friend.interface.components.monitoring import CameraPosition
from field_friend.localization import GeoPoint
import shapely
if TYPE_CHECKING:
    from field_friend.system import System


class SupportPointDialog:

    def __init__(self, system: 'System'):
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.steerer = system.steerer
        self.gnss = system.gnss
        self.field_provider = system.field_provider
        self.row_name: int = 1
        self.support_point_coordinates: GeoPoint | None = None
        self.next: Callable = self.find_support_point

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

    def find_support_point(self) -> None:
        self.headline.text = 'Drive to First Row'
        self.row_sight.content = '<line x1="50%" y1="0" x2="50%" y2="100%" stroke="#6E93D6" stroke-width="6"/>'
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('1. Drive the robot on the row you want to give a fixed support point.').classes(
                'text-lg')
            ui.label('2. Enter the row number for the support point:').classes('text-lg')
            ui.number(
                label='Row Number', min=1, max=self.field_provider.fields[0].row_number - 1 if self.field_provider.fields else 1, step=1, value=1) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Choose the row number you would like to give a fixed support point to.') \
                .bind_value(self, 'row_name')
        self.next = self.confirm_support_point

    def confirm_support_point(self) -> None:
        assert self.gnss.current is not None
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.support_point_coordinates = self.gnss.current.location
        self.headline.text = 'Confirm Values'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                ui.label(f'Support Point Coordinates: {self.support_point_coordinates}').classes('text-lg')
                ui.label(f'Row Number: {round(self.row_name)}').classes('text-lg')
            with ui.row().classes('items-center'):
                ui.button('Cancel', on_click=self.dialog.close).props('color=red')
        self.next = self._apply

    def _apply(self) -> None:
        self.dialog.close()
        if self.support_point_coordinates is None:
            ui.notify('No valid point coordinates.')
            return
        row_index = self.row_name - 1
        field = self.field_provider.fields[0]

        first_row_start = field.first_row_start.cartesian()
        first_row_end = field.first_row_end.cartesian()
        first_row_line = shapely.geometry.LineString(
            [[first_row_start.x, first_row_start.y], [first_row_end.x, first_row_end.y]])
        support_point = self.support_point_coordinates.cartesian()
        distance = first_row_line.distance(shapely.geometry.Point([support_point.x, support_point.y]))
        row_support_point = RowSupportPoint(row_index=row_index, distance=distance)
        self.field_provider.add_row_support_point(field.id, row_support_point)
        self.first_row_start = None
        self.first_row_end = None

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
