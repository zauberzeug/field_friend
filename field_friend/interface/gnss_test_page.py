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

        self.system.gnss.NEW_MEASUREMENT.register(self.on_new_measurement)

    def ui(self) -> None:
        with ui.row().classes('w-1/3 h-1/3 aspect-square'):
            self.echart = ui.echart({
                'xAxis': {'type': 'value', 'min': -3, 'max': 3},
                'yAxis': {'type': 'value', 'min': -3, 'max': 3},
                'series': [
                    {
                        'symbolSize': 2,
                        'data': [],
                        'z': 1,
                        'type': 'scatter'
                    },
                ]
            }).classes('w-full h-full')
            ui.button('Clear').on_click(self.clear)

    def on_new_measurement(self, measurement: GnssMeasurement) -> None:
        self.points.append(measurement.pose.point.to_local())
        if self.echart is None:
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
        self.echart.update()

    def clear(self) -> None:
        self.points = []
        if self.echart is None:
            return
        self.echart.options['series'][0]['data'] = []
        self.echart.update()
