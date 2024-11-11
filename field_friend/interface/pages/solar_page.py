from typing import TYPE_CHECKING

import rosys
from nicegui import ui

from ... import localization
from ..components import camera_card

if TYPE_CHECKING:
    from field_friend.system import System


@ui.refreshable
def gnss_ui(system: 'System') -> None:
    direction_flag = '?' if system.gnss.current is None or system.gnss.current.heading is None else \
        'N' if system.gnss.current.heading <= 23 else \
        'NE' if system.gnss.current.heading <= 68 else \
        'E' if system.gnss.current.heading <= 113 else \
        'SE' if system.gnss.current.heading <= 158 else \
        'S' if system.gnss.current.heading <= 203 else \
        'SW' if system.gnss.current.heading <= 248 else \
        'W' if system.gnss.current.heading <= 293 else \
        'NW' if system.gnss.current.heading <= 338 else \
        'N'

    ui.label('Positioning').classes('w-full text-center text-bold')
    ui.separator()
    with ui.row().classes('place-items-center'):
        ui.label('GNSS-Device:').classes('text-bold')
        ui.label('No connection' if system.gnss.device is None else 'Connected')
    with ui.row().classes('place-items-center'):
        ui.label('Reference position:').classes('text-bold')
        ui.label('No reference' if localization.reference is None or (localization.reference.lat ==
                                                                      0 and localization.reference.long == 0) else str(localization.reference))
    with ui.row().classes('place-items-center'):
        ui.label('Position:').classes('text-bold')
        ui.label('No location' if system.gnss.current is None else f'{system.gnss.current.location}°')
    with ui.row().classes('place-items-center'):
        ui.label('StdDev:').classes('text-bold')
        ui.label('No location' if system.gnss.current is None else
                 f'({system.gnss.current.latitude_std_dev:.4f}, {system.gnss.current.longitude_std_dev:.4f})')
    with ui.row().classes('place-items-center'):
        ui.label('Heading:').classes('text-bold')
        ui.label(f'{system.gnss.current.heading:.2f}° \
                 {direction_flag}' if system.gnss.current is not None and system.gnss.current.heading is not None else 'No heading')
    with ui.row().classes('place-items-center'):
        ui.label('RTK-Fix:').classes('text-bold')
        ui.label(
            f'gps_qual: {system.gnss.current.gps_qual}, mode: {system.gnss.current.mode}' if system.gnss.current is not None else 'No fix')
    with ui.row().classes('place-items-center'):
        ui.label('Satellites:').classes('text-bold')
        ui.label(f'{system.gnss.current.num_sats}' if system.gnss.current is not None else 'No satellites')
    with ui.row().classes('place-items-center'):
        ui.label('Odometry:').classes('text-bold')
        ui.label(str(system.odometer.prediction))


class solar_page():
    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system
        self.gnss = system.gnss
        self.laser_scanner = system.laser_scanner
        self.camera_provider = system.camera_provider
        self.file_recorder = system.file_recorder

        @ui.page('/solar')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        # with ui.row():
        #     self.recorder.ui()
        with ui.row():
            with ui.column().classes('w-128'):
                with ui.card().classes('w-full'):
                    refreshable = ui.refreshable(gnss_ui)
                    refreshable(self.system)
                    ui.timer(rosys.config.ui_update_interval, lambda: refreshable.refresh(self.system))
                with ui.card().classes('w-full'):
                    self.file_recorder.recorder_ui()
            with ui.card().tight():
                self.laser_scanner.ui()
            with ui.card().tight().classes('w-auto'):
                camera_card(self.system, shrink_factor=2)
