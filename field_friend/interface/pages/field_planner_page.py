

from typing import TYPE_CHECKING, Any, Literal, TypedDict

from nicegui import binding, ui

from ...automations import Field, FieldObstacle, Row
from ..components import field_planner, leaflet_map

if TYPE_CHECKING:
    from field_friend.system import System


class Active_object(TypedDict):
    object_type: Literal["Obstacles", "Rows", "Outline"]
    object: Row | FieldObstacle


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
                leaflet = leaflet_map(self.system, True)
                leaflet.m.style('height: 100%; max-height:100%;')
                with ui.column():
                    leaflet.buttons()
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height: 60%; max-height:60%;'):
                planner = field_planner(self.system, leaflet)
                binding.bind_to(planner, 'active_field', leaflet, 'active_field', lambda f: f.id if f else None)
                binding.bind_to(planner, 'active_object', leaflet, 'active_object', lambda o: o if o else None)
