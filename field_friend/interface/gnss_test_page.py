from __future__ import annotations

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
                        'symbolSize': 5,
                        'data': [],
                        'type': 'scatter'
                    }
                ]
            }).classes('w-full h-full')
            ui.button('Clear').on_click(self.clear)

    def on_new_measurement(self, measurement: GnssMeasurement) -> None:
        self.points.append(measurement.pose.point.to_local())
        if self.echart is None:
            return
        self.echart.options['series'][0]['data'] = [point.tuple for point in self.points]
        self.echart.update()

    def clear(self) -> None:
        self.points = []
        if self.echart is None:
            return
        self.echart.options['series'][0]['data'] = []
        self.echart.update()
