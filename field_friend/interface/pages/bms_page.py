from __future__ import annotations

import re
from datetime import datetime
from typing import TYPE_CHECKING

import pandas as pd
import rosys
from nicegui import ui

from ...log_configuration import PATH as LOG_PATH

if TYPE_CHECKING:
    from system import System


class bms_page:

    def __init__(self, page_wrapper, system: System) -> None:
        self.system = system

        @ui.page('/bms')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        with ui.row().classes('w-full'):
            with ui.column().classes('col-12'):
                self._bms_history_section()
        with ui.row().classes('w-full gap-0'):
            with ui.column().classes('col-4'):
                self._bms_section()

    def _bms_section(self) -> None:
        ui.label('BMS Overview').classes('text-2xl')

        field_friend = self.system.field_friend
        bms = field_friend.bms

        @ui.refreshable
        def show_bms_data():
            ui.json_editor({
                'content': {'json': bms.raw_data},
                'mode': 'text',
                'mainMenuBar': False,
                'navigationBar': False,
                'statusBar': False,
                'readOnly': True,
            })
        show_bms_data()
        ui.timer(1.0, show_bms_data.refresh)

        if self.system.is_real:
            assert isinstance(bms, rosys.hardware.BmsHardware)
            assert isinstance(field_friend, rosys.hardware.RobotHardware)
            msg_0x04 = f'{bms.name}.send(0xdd, 0xa5, 0x04, 0x00, 0xff, 0xfc, 0x77)'
            ui.button('Request Voltages', on_click=lambda: field_friend.robot_brain.send(msg_0x04))
            msg_0x05 = f'{bms.name}.send(0xdd, 0xa5, 0x05, 0x00, 0xff, 0xfb, 0x77)'
            ui.button('Request Version', on_click=lambda: field_friend.robot_brain.send(msg_0x05))

    def _bms_history_section(self) -> None:
        ui.label('BMS History').classes('text-2xl')

        log_lines = []
        for path in LOG_PATH.glob('battery.log*'):
            log_lines += path.read_text().strip().split('\n')

        log_entries = []
        pattern = re.compile(
            r'(\d+-\d+-\d+ \d+:\d+:\d+\.\d+).*Battery: (\d+\.?\d*)%, (\d+\.?\d*)V, (-?\d+\.?\d*)A, (\d+\.?\d*)°C')
        for line in sorted(log_lines):
            if match := pattern.search(line):
                timestamp = datetime.strptime(match.group(1), r'%Y-%m-%d %H:%M:%S.%f')
                battery = float(match.group(2))
                voltage = float(match.group(3))
                current = float(match.group(4))
                temperature = float(match.group(5))
                log_entries.append((timestamp, battery, voltage, current, temperature))

        if not log_entries:
            ui.label('No log data available')
            return

        df = pd.DataFrame(log_entries, columns=['Timestamp', 'Battery', 'Voltage', 'Current', 'Temperature'])
        t = [ts.to_pydatetime() for ts in df['Timestamp']]

        ui.echart({
            'tooltip': {'trigger': 'axis'},
            'xAxis': {'type': 'time', 'data': t},
            'yAxis': {'type': 'value', 'min': -5, 'max': 100},
            'series': [
                {'type': 'line', 'showSymbol': False, 'name': 'Battery (%)',
                 'data': [list(a) for a in zip(t, df['Battery'].to_list(), strict=True)]},
                {'type': 'line', 'showSymbol': False, 'name': 'Voltage (V)',
                 'data': [list(a) for a in zip(t, df['Voltage'].to_list(), strict=True)]},
                {'type': 'line', 'showSymbol': False, 'name': 'Current (A)',
                 'data': [list(a) for a in zip(t, df['Current'].to_list(), strict=True)]},
                {'type': 'line', 'showSymbol': False, 'name': 'Temperature (°C)',
                 'data': [list(a) for a in zip(t, df['Temperature'].to_list(), strict=True)]},
            ],
        }).classes('w-full')
