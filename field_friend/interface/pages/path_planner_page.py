from typing import TYPE_CHECKING

from nicegui import ui

from ..components import LeafletMap as leaflet_map
from ..components import PathPlanner as path_planner
from ..components import operation

if TYPE_CHECKING:
    from ...system import System


class path_planner_page:

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/path')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self):
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.column().classes('h-full p-2').style('width: 100%;'):
            leaflet_map_path = leaflet_map(self.system, False)
            leaflet_map_path.m.classes(
                'h-full w-full')
        with ui.column().classes('w-full items-stretch'):
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                operation(self.system)
                path_planner(self.system.path_provider, self.system.path_recorder, self.system.automator)
