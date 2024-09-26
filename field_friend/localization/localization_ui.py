from typing import TYPE_CHECKING

from nicegui import ui

from .. import localization

if TYPE_CHECKING:
    from field_friend.system import System


@ui.refreshable
def developer_ui(system: 'System') -> None:
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
        ui.label(f'{system.gnss.current.heading:.2f}° {direction_flag}' if system.gnss.current is not None and system.gnss.current.heading is not None else 'No heading')
    with ui.row().classes('place-items-center'):
        ui.label('Heading:').classes('text-bold')
        ui.label(f'{system.gnss.current.heading:.2f}° {direction_flag}' if system.gnss.current is not None and system.gnss.current.heading is not None else 'No heading')
    with ui.row().classes('place-items-center'):
        ui.label('RTK-Fix:').classes('text-bold')
        ui.label(
            f'gps_qual: {system.gnss.current.gps_qual}, mode: {system.gnss.current.mode}' if system.gnss.current is not None else 'No fix')
    with ui.row().classes('place-items-center'):
        ui.label('GNSS paused:').classes('text-bold')
        ui.label(str(system.gnss.is_paused))
    with ui.row().classes('place-items-center'):
        ui.label('Odometry:').classes('text-bold')
        ui.label(str(system.odometer.prediction))
