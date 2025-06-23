# pylint: disable=duplicate-code
# TODO: refactor this and field_creator.py
from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import rosys
from nicegui import ui
from rosys.geometry import GeoPoint
from rosys.hardware.gnss import GpsQuality

from ...automations.field import RowSupportPoint

if TYPE_CHECKING:
    from ...system import System


class SupportPointDialog:

    def __init__(self, system: System) -> None:
        self.front_cam: rosys.vision.MjpegCamera | None = None
        if hasattr(system, 'mjpeg_camera_provider') and system.config.circle_sight_positions is not None:
            self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                                   if system.config.circle_sight_positions.front in key), None)
        self.steerer = system.steerer
        self.gnss = system.gnss
        self.field_provider = system.field_provider
        self.row_name: int = 0
        self.bed_number: int = 0
        self.support_point_coordinates: GeoPoint | None = None
        self.next: Callable = self.find_support_point

        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                self.row_sight = ui.interactive_image().classes('w-3/5')
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    ui.label('No GNSS RTK fix available.').classes('text-red') \
                        .bind_visibility_from(self.gnss, 'last_measurement', lambda m: m.gps_quality != GpsQuality.RTK_FIXED)
                    # NOTE: the next function is replaced, hence we need the lambda
                    # pylint: disable=unnecessary-lambda
                    ui.button('Next', on_click=lambda: self.next()) \
                        .bind_enabled_from(self.gnss, 'last_measurement', lambda m: m.gps_quality == GpsQuality.RTK_FIXED)
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
            ui.label('1. Drive the robot on the row you want to give a fixed support point.').classes('text-lg')
            if self.field_provider.selected_field and self.field_provider.selected_field.source.bed_count == 1:
                ui.label('2. Enter the row number for the support point:').classes('text-lg')
                ui.select(list(range(self.field_provider.selected_field.source.row_count)), label='Row') \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the row index you would like to give a fixed support point to.') \
                    .bind_value(self, 'row_name')
            elif self.field_provider.selected_field is not None:
                ui.label('2. Enter the bed and row number for the support point:').classes('text-lg')
                ui.select(list(range(self.field_provider.selected_field.source.bed_count)), label='Bed') \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the bed index the row is on.') \
                    .bind_value(self, 'bed_number')
                ui.select(list(range(self.field_provider.selected_field.source.row_count)), label='Bed Row') \
                    .props('dense outlined').classes('w-40') \
                    .tooltip('Choose the row index you would like to give a fixed support point to.') \
                    .bind_value(self, 'row_name')
        self.next = self.confirm_support_point

    def confirm_support_point(self) -> None:
        def refresh_position() -> None:
            assert self.gnss is not None
            assert self.gnss.last_measurement is not None
            self.support_point_coordinates = self.gnss.last_measurement.point
        self.content.clear()
        self.headline.text = 'Confirm Values'
        refresh_position()
        with self.content:
            with ui.row().classes('items-center'):
                with ui.column():
                    ui.label('').classes('text-lg').bind_text_from(self, 'support_point_coordinates', lambda p: f'{p}')
                    if self.field_provider.selected_field and self.field_provider.selected_field.source.bed_count > 1:
                        ui.label(f'Bed Number: {round(self.bed_number)}').classes('text-lg')
                    ui.label(f'Row Number: {round(self.row_name)}').classes('text-lg')
            with ui.row().classes('items-center'):
                ui.button('Refresh Position', on_click=refresh_position) \
                    .bind_enabled_from(self.gnss, 'last_measurement', lambda m: m.gps_quality == GpsQuality.RTK_FIXED)
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
        row_index = self.bed_number * field.source.row_count + self.row_name
        row_support_point = RowSupportPoint.from_geopoint(self.support_point_coordinates, row_index, waypoint_index=0)
        self.field_provider.add_row_support_point(field.source.id, row_support_point)
        ui.notify('Support point added.')

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.front_cam.streaming = True
        self.row_sight.set_source(self.front_cam.get_latest_image_url())
