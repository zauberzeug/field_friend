from __future__ import annotations

import numpy as np
from nicegui import ui
from rosys.geometry import Point
from rosys.hardware import GnssMeasurement

from ..system import System
from .components import create_header


class GnssTestPage:

    def __init__(self, system: System) -> None:
        self.system = system

        @ui.page('/gnss_test')
        def page() -> None:
            create_header(system)
            self.ui()

        self.echart: ui.ECharts | None = None
        self.points: list[Point] = []
        self.range: int = 3
        self.max_length: int = 3000
        self.label: ui.label | None = None

        assert self.system.gnss is not None
        self.system.gnss.NEW_MEASUREMENT.register(self.on_new_measurement)

    def ui(self) -> None:
        with ui.row().classes('w-1/3 h-1/3 aspect-square'):
            self.echart = ui.echart({
                'xAxis': {'type': 'value', 'min': -self.range, 'max': self.range},
                'yAxis': {'type': 'value', 'min': -self.range, 'max': self.range},
                'series': [
                    {
                        'symbolSize': 2,
                        'data': [],
                        'z': 1,
                        'type': 'scatter'
                    },
                ]
            }).classes('w-full h-full')
            self.label = ui.label('0')
            ui.button('Clear').on_click(self.clear)
            ui.number('Range', on_change=self.on_range_change).bind_value(self, 'range')

    def on_new_measurement(self, measurement: GnssMeasurement) -> None:
        if len(self.points) >= self.max_length:
            return
        self.points.append(measurement.pose.point.to_local())
        if self.echart is None or self.label is None:
            return

        points_array = np.array([(point.x, point.y) for point in self.points])
        mean_x, mean_y = np.mean(points_array, axis=0) if len(self.points) > 0 else (0, 0)
        center_data = {
            'value': [mean_x, mean_y],
            'symbolSize': 10,
            'z': 100,
            'itemStyle': {
                'color': 'red'
            }
        }
        self.echart.options['series'][0]['data'] = [center_data] + [point.tuple for point in self.points]
        self.label.text = f'{len(self.points)}'
        self.echart.update()

    def clear(self) -> None:
        self.points = []
        if self.echart is None:
            return
        self.echart.options['series'][0]['data'] = []
        self.echart.update()

    def on_range_change(self) -> None:
        if self.echart is None:
            return
        self.echart.options['xAxis']['min'] = -self.range
        self.echart.options['xAxis']['max'] = self.range
        self.echart.options['yAxis']['min'] = -self.range
        self.echart.options['yAxis']['max'] = self.range
        self.echart.update()
