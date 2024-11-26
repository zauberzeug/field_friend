from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING
from uuid import uuid4
import rosys
from nicegui import ui
from field_friend.automations.field import Field
from field_friend.interface.components.monitoring import CameraPosition
from field_friend.localization import GeoPoint
from nicegui.elements.leaflet_layers import Marker
if TYPE_CHECKING:
    from ...system import System


class FieldCreator:

    def __init__(self, system: System) -> None:
        self.system = system
        self.front_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                               if CameraPosition.FRONT in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.back_cam = next((value for key, value in system.mjpeg_camera_provider.cameras.items()
                              if CameraPosition.BACK in key), None) if hasattr(system, 'mjpeg_camera_provider') else None
        self.steerer = system.steerer
        self.gnss = system.gnss
        self.plant_locator = system.plant_locator
        self.field_provider = system.field_provider
        self.first_row_start: GeoPoint | None = None
        self.first_row_end: GeoPoint | None = None
        self.field_name: str = 'Field'
        self.row_spacing: float = 0.5
        self.row_count: int = 10
        self.outline_buffer_width: float = 2.0
        self.bed_count: int = 1
        self.bed_spacing: float = 0.5
        self.bed_crops: dict[str, str | None] = {'0': None}
        self.next: Callable = self.find_first_row
        self.default_crop: str | None = None
        self.m: ui.leaflet
        self.robot_marker: Marker | None = None
        self.gnss.ROBOT_GNSS_POSITION_CHANGED.register_ui(self.update_robot_position)
        with ui.dialog() as self.dialog, ui.card().style('width: 900px; max-width: none'):
            with ui.row().classes('w-full no-wrap no-gap'):
                with ui.column().classes('w-3/5') as self.view_column:
                    self.row_sight = ui.interactive_image().classes('w-full')
                    self.camera_updater = ui.timer(0.1, self.update_front_cam)
                with ui.column().classes('items-center  w-2/5 p-8'):
                    self.headline = ui.label().classes('text-lg font-bold')
                    self.content = ui.column().classes('items-center')
                    # NOTE: the next function is replaced, hence we need the lambda
                    ui.button('Next', on_click=lambda: self.next())  # pylint: disable=unnecessary-lambda
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
        self.next = self.find_row_ending

    def find_row_ending(self) -> None:
        assert self.gnss.current is not None
        if not ('R' in self.gnss.current.mode or self.gnss.current.mode == 'SSSS'):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_start = self.gnss.current.location
        assert self.first_row_start is not None
        # TODO: save the point somewhere to be able to use it in case of restarting the creator
        self.view_column.clear()
        with self.view_column:
            self.row_sight = ui.interactive_image().classes('w-full')
            self.row_sight.content = '<line x1="50%" y1="0" x2="50%" y2="100%" stroke="#6E93D6" stroke-width="6"/>'
            self.camera_updater = ui.timer(0.1, self.update_back_cam)
        self.headline.text = 'Find Row Ending'
        self.content.clear()
        with self.content:
            rosys.driving.joystick(self.steerer, size=50, color='#6E93D6')
            ui.label('1. Drive the robot to the end of the current row.') \
                .classes('text-lg')
            ui.label('2. Place the robot about 1 meter after the last crop.') \
                .classes('text-lg')
        self.next = self.field_infos

    def field_infos(self) -> None:
        assert self.gnss.current is not None
        if not ("R" in self.gnss.current.mode or self.gnss.current.mode == "SSSS"):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_end = self.gnss.current.location
        assert self.first_row_end is not None
        self.view_column.clear()
        with self.view_column:
            self.ab_line_map()
        self.headline.text = 'Field Parameters'
        self.row_sight.content = ''
        self.content.clear()
        with self.content:
            ui.input('Field Name') \
                .props('dense outlined').classes('w-40') \
                .tooltip('Enter a name for the field') \
                .bind_value(self, 'field_name')
            ui.separator()
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
            ui.select(label='Default Crop', options=self.plant_locator.crop_category_names) \
                .props('dense outlined').classes('w-40') \
                .tooltip('Enter the default crop for all beds') \
                .bind_value(self, 'default_crop')
            ui.separator()
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
        self.next = self.crop_infos

    def crop_infos(self) -> None:
        assert self.gnss.current is not None
        if not ('R' in self.gnss.current.mode or self.gnss.current.mode == 'SSSS'):
            with self.content:
                ui.label('No RTK fix available.').classes('text-red')
        self.first_row_end = self.gnss.current.location
        assert self.first_row_end is not None

        self.headline.text = 'Crops'
        self.content.clear()
        with self.content:
            for i in range(int(self.bed_count)):
                with ui.row().classes('w-full'):
                    ui.label(f'Bed {i + 1}:').classes('text-lg')
                    ui.select(options=self.plant_locator.crop_category_names) \
                        .props('dense outlined').classes('w-40') \
                        .tooltip(f'Enter the crop name for bed {i + 1}') \
                        .bind_value(self, 'bed_crops',
                                    forward=lambda v, idx=i: {**self.bed_crops,
                                                              str(idx): v if v is not None else self.default_crop},
                                    backward=lambda v, idx=i: v.get(str(idx)))

        self.next = self.confirm_geometry

    def confirm_geometry(self) -> None:
        self.headline.text = 'Confirm Geometry'
        self.content.clear()
        with self.content.style('max-height: 100%; overflow-y: auto'):
            with ui.row().classes('items-center'):
                ui.label(f'Field Name: {self.field_name}').classes('text-lg')
                ui.separator()
                if self.bed_count > 1:
                    ui.label(f'Number of Beds: {int(self.bed_count)}').classes('text-lg')
                    ui.label(f'Bed Spacing: {self.bed_spacing*100} cm').classes('text-lg')
                with ui.expansion('Crops').classes('w-full'):
                    for i in range(int(self.bed_count)):
                        crop = self.bed_crops[str(i)]
                        crop_name = self.plant_locator.crop_category_names[crop] if crop is not None else 'No crop selected'
                        ui.label(f'Bed {int(i) + 1}: {crop_name}').classes('text-lg')
                ui.separator()
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
                                                   bed_spacing=self.bed_spacing,
                                                   bed_crops=self.bed_crops))
        else:
            self.field_provider.create_field(Field(id=str(uuid4()),
                                                   name=self.field_name,
                                                   first_row_start=self.first_row_start,
                                                   first_row_end=self.first_row_end,
                                                   row_spacing=self.row_spacing,
                                                   row_count=int(self.row_count),
                                                   outline_buffer_width=self.outline_buffer_width,
                                                   bed_crops=self.bed_crops))
        self.first_row_start = None
        self.first_row_end = None

    def update_front_cam(self) -> None:
        if self.front_cam is None:
            return
        self.row_sight.set_source(self.front_cam.get_latest_image_url())

    def update_back_cam(self) -> None:
        if self.back_cam is None:
            return
        self.row_sight.set_source(self.back_cam.get_latest_image_url())

    def ab_line_map(self) -> None:
        if self.gnss.current is None:
            return
        self.m = ui.leaflet(self.gnss.current.location.tuple).classes('w-full min-h-[500px]')
        if self.first_row_start is not None and self.first_row_end is not None:
            self.m.generic_layer(name='polyline', args=[
                (self.first_row_start.tuple, self.first_row_end.tuple), {'color': '#F44336'}])
        self.m.set_zoom(18)

    def update_robot_position(self, position: GeoPoint, dialog=None) -> None:
        if hasattr(self, 'm') and self.m and isinstance(self.m, ui.leaflet):
            self.robot_marker = self.robot_marker or self.m.marker(latlng=position.tuple)
            icon = 'L.icon({iconUrl: "assets/robot_position_side.png", iconSize: [24,24], iconAnchor:[12,12]})'
            self.robot_marker.run_method(':setIcon', icon)
            self.robot_marker.move(*position.tuple)
