from nicegui import ui
from ..components import header_bar, system_bar, field_planner, leaflet_map, status_drawer

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from field_friend.system import System


class field_planner_page():

    def __init__(self, system: 'System') -> None:

        @ui.page('/field')
        def field_page():
            ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
            drawer = status_drawer(system, system.field_friend, system.gnss, system.odometer)
            header_bar(system, drawer)
            system_bar()
            with ui.column().classes('w-full items-stretch').style('max-height:calc(100vh - 125px); height:calc(100vh - 150px);'):
                with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height:40%; max-height:40%;'):
                    leaflet_map_field = leaflet_map(system, True)
                    leaflet_map_field.m.style('height: 100%; max-height:100%;')
                with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height: 60%; max-height:60%;'):
                    field_planner(system.field_provider, system.odometer, system.gnss, leaflet_map_field)
