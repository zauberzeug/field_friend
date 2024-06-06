

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
        self.active_field: Field | None = None
        self.active_object: Active_object | None = None
        # TODO: hier noch die beiden Variablen abspeichern für Frontend persistenz

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
                if self.active_field is not None:
                    leaflet.active_field = self.active_field.id
                binding.bind_to(self, 'active_field', leaflet, 'active_field',
                                lambda f: f.id if f else None)
                binding.bind_to(self, 'active_object', leaflet, 'active_object',
                                lambda f: f if f else None)
                with ui.column():
                    leaflet.buttons()
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap; height: 60%; max-height:60%;'):
                self.field_planner = field_planner(self.active_field, self.active_object, self.system.field_provider,
                                                   self.system.odometer, self.system.gnss, leaflet)
                binding.bind_to(self.field_planner, 'active_field', self, 'active_field',
                                lambda f: f if f else None)
                binding.bind_to(self.field_planner, 'active_object', self, 'active_object',
                                lambda f: f if f else None)
