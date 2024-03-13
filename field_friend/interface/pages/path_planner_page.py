from typing import TYPE_CHECKING

from nicegui import ui

from ..components import header_bar, leaflet_map, operation, path_planner, status_drawer, system_bar

if TYPE_CHECKING:
    from field_friend.system import System


class path_planner_page():

    def __init__(self, system: 'System') -> None:
        @ui.page('/path')
        def path_page():
            ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
            drawer = status_drawer(system, system.field_friend, system.gnss, system.odometer)
            header_bar(system, drawer)
            system_bar()
            with ui.column().classes('h-full p-2').style('width: 100%;'):
                leaflet_map_path = leaflet_map(system, False)
                leaflet_map_path.m.classes(
                    'h-full w-full')
            with ui.column().classes('w-full items-stretch'):
                with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                    operation(system, leaflet_map_path)
                    path_planner(system.path_provider, system.path_recorder, system.automator)
