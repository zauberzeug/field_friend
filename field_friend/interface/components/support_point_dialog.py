# pylint: disable=duplicate-code
# TODO: refactor this and field_creator.py
from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.geometry import GeoPoint

from ...automations.field import RowSupportPoint
from ...interface.components.monitoring import CameraPosition

if TYPE_CHECKING:
    from ...system import System


class SupportPointDialog:

    def __init__(self, system: System) -> None:
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.steerer = system.steerer
        self.gnss = system.gnss
        self.field_provider = system.field_provider
        self.row_name: int = 1
        self.bed_number: int = 1
        self.support_point_coordinates: GeoPoint | None = None
        self.next: Callable = self.find_support_point

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

    def find_support_point(self) -> None:
        self.headline.text = 'Drive to Row'
        self.row_sight.content = '<line x1="50%" y1="0" x2="50%" y2="100%" stroke="#6E93D6" stroke-width="6"/>'
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('1. Drive the robot on the row you want to give a fixed support point.').classes(
                'text-lg')
            if self.field_provider.selected_field and self.field_provider.selected_field.bed_count == 1:
                ui.label('2. Enter the row number for the support point:').classes('text-lg')
                ui.number(
                    label='Row Number', min=1, max=self.field_provider.selected_field.row_count, step=1, value=1) \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the row number you would like to give a fixed support point to.') \
                    .bind_value(self, 'row_name')
            elif self.field_provider.selected_field is not None:
                ui.label('2. Enter the bed and row number for the support point:').classes('text-lg')
                ui.number(
                    label='Bed Number', min=1, max=self.field_provider.selected_field.bed_count, step=1, value=1) \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the bed number the row is on.') \
                    .bind_value(self, 'bed_number')
                ui.number(
                    label='Row Number', min=1, max=self.field_provider.selected_field.row_count, step=1, value=1) \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the row number you would like to give a fixed support point to.') \
                    .bind_value(self, 'row_name')
        self.next = self.confirm_support_point

    def confirm_support_point(self) -> None:
        assert self.gnss.last_measurement is not None
        # TODO: use gps quality instead
        # if not ('R' in self.gnss.last_measurement.mode or self.gnss.last_measurement.mode == 'SSSS'):
        #     with self.content:
        #         ui.label('No RTK fix available.').classes('text-red')
        self.support_point_coordinates = self.gnss.last_measurement.point
        self.headline.text = 'Confirm Values'
        self.content.clear()
        with self.content:
            with ui.row().classes('items-center'):
                ui.label(f'Support Point Coordinates: {self.support_point_coordinates}').classes('text-lg')
                ui.label(f'Row Number: {round(self.row_name)}').classes('text-lg')
                ui.label(f'Bed Number: {round(self.bed_number)}').classes('text-lg')
            with ui.row().classes('items-center'):
                ui.button('Cancel', on_click=self.dialog.close).props('color=red')
        self.next = self._apply

    def _apply(self) -> None:
        self.dialog.close()
        if self.support_point_coordinates is None:
            ui.notify('No valid point coordinates.')
            return
        field = self.field_provider.selected_field
        if field is None:
            ui.notify('No field selected.')
            return
        row_index = (self.bed_number - 1) * field.row_count + self.row_name - 1
        row_support_point = RowSupportPoint.from_geopoint(self.support_point_coordinates, row_index)
        self.field_provider.add_row_support_point(field.id, row_support_point)
        ui.notify('Support point added.')

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
