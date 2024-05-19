from typing import TYPE_CHECKING

from nicegui import ui

from ..components import field_planner, leaflet_map

if TYPE_CHECKING:
    from field_friend.system import System


class field_planner_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/field')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.column().classes('w-full items-stretch').style('max-height:calc(100vh - 125px); height:calc(100vh - 150px);'):
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height:40%; max-height:40%;'):
                leaflet_map_field = leaflet_map(self.system, True)
                leaflet_map_field.m.style('height: 100%; max-height:100%;')
                with ui.column():
                    leaflet_map_field.buttons()
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height: 60%; max-height:60%;'):
                field_planner(self.system.field_provider, self.system.odometer, self.system.gnss, leaflet_map_field)
