import rosys
from nicegui import ui
from rosys.analysis import track

from ..system import System
from .components import Operation as operation
from .components import create_header
from .components.log_monitor import LogMonitor


class LowBandwidthPage:

    def __init__(self, system: System) -> None:
        self.system = system
        self.log_monitor = LogMonitor()

        @ui.page('/lb')
        def page() -> None:
            create_header(system)
            self.content(devmode=False)

    def content(self, devmode: bool) -> None:
        page_height = '50vh' if devmode else 'calc(100vh - 170px)'
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.row().style(f'height:{page_height}; width: calc(100vw - 2rem); flex-wrap: nowrap;'):
            operation(self.system)
            with ui.column().classes('h-full').style('width: calc(45% - 2rem); flex-wrap: nowrap;'):
                with ui.row().style('margin: 1rem; width: calc(100% - 2rem);'):
                    with ui.column():
                        ui.button('emergency stop', on_click=lambda: self.system.field_friend.estop.set_soft_estop(True)).props('color=red') \
                            .classes('py-3 px-6 text-lg').bind_visibility_from(self.system.field_friend.estop, 'is_soft_estop_active', value=False)
                        ui.button('emergency reset', on_click=lambda: self.system.field_friend.estop.set_soft_estop(False)) \
                            .props('color=red-700 outline').classes('py-3 px-6 text-lg') \
                            .bind_visibility_from(self.system.field_friend.estop, 'is_soft_estop_active', value=True)
                        with ui.row():
                            rosys.automation.automation_controls(self.system.automator)
                    with ui.column().bind_visibility_from(self.system.automator, 'is_running'):
                        ui.label('').bind_text_from(self.system.current_navigation,
                                                    '_state', lambda state: f'State: {state.name}')
                        ui.label('').bind_text_from(self.system.current_navigation,
                                                    'row_index', lambda row_index: f'Row Index: {row_index}')
            with ui.column():
                with ui.row().style('min-height: 37px'):
                    track.ui()
                self.log_monitor.ui()
