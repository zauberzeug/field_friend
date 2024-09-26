import datetime
from typing import TYPE_CHECKING

from nicegui import ui

if TYPE_CHECKING:
    from field_friend.system import System


@ui.refreshable
def developer_ui(system: 'System') -> None:
    ui.label('Battery').classes('w-full text-center text-bold')
    ui.separator()
    with ui.row().classes('place-items-center'):
        ui.label('Percentage:').classes('text-bold')
        ui.label(
            'No Data' if system.field_friend.bms.state.percentage is None else f'{system.field_friend.bms.state.percentage:.1f}%')
    with ui.row().classes('place-items-center'):
        ui.label('Voltage:').classes('text-bold')
        ui.label(
            'No Data' if system.field_friend.bms.state.voltage is None else f'{system.field_friend.bms.state.voltage:.1f}V')
    with ui.row().classes('place-items-center'):
        ui.label('Current:').classes('text-bold')
        ui.label(
            'No Data' if system.field_friend.bms.state.current is None else f'{system.field_friend.bms.state.current:.1f}A')
    with ui.row().classes('place-items-center'):
        ui.label('Temperature:').classes('text-bold')
        ui.label(
            'No Data' if system.field_friend.bms.state.temperature is None else f'{system.field_friend.bms.state.temperature:.1f}°C')
    with ui.row().classes('place-items-center'):
        ui.label('Is Charging:').classes('text-bold')
        ui.label('No Data' if system.field_friend.bms.state.is_charging is None else str(
            system.field_friend.bms.state.is_charging))
    with ui.row().classes('place-items-center'):
        ui.label('Last Update:').classes('text-bold')
        ui.label('No Data' if system.field_friend.bms.state.last_update is None else str(
            datetime.datetime.fromtimestamp(system.field_friend.bms.state.last_update).strftime("%H:%M:%S.%f")[:-3]))
    with ui.row().classes('place-items-center'):
        ui.label('Battery Control:').classes('text-bold')
        if hasattr(system.field_friend, 'battery_control') and system.field_friend.battery_control is not None:
            ui.label('Out 1..4 is on' if system.field_friend.battery_control.status else 'Out 1..4 is off')
        else:
            ui.label('No Data')
