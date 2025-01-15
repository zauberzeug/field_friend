import rosys
from nicegui import binding, ui

from ..system import System
from .components import CameraCard as camera_card
from .components import LeafletMap as leaflet_map
from .components import Operation as operation
from .components import RobotScene as robot_scene
from .components import create_header


class MainPage:

    def __init__(self, system: System) -> None:
        self.system = system

        @ui.page('/')
        def page() -> None:
            create_header(system)
            self.content(devmode=False)

    def content(self, devmode: bool) -> None:
        page_height = '50vh' if devmode else 'calc(100vh - 170px)'
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.row().style(f'height:{page_height}; width: calc(100vw - 2rem); flex-wrap: nowrap;'):
            with ui.column().classes('h-full w-1/2 p-2'):
                leaflet = leaflet_map(self.system, False)
                leaflet.m.classes('h-full w-full')
                binding.bind_to(self.system.field_navigation, 'field', leaflet, 'active_field',
                                lambda f: f.id if f else None)
                with ui.row():
                    leaflet.buttons()
            with ui.row().classes('h-full ml-2 m-2').style('width: calc(100% - 1rem)'):
                operation(self.system)
                with ui.column().classes('h-full').style('width: calc(45% - 2rem); flex-wrap: nowrap;'):
                    camera_card(self.system)
                    robot_scene(self.system)
                    with ui.row().style('margin: 1rem; width: calc(100% - 2rem);'):
                        with ui.column():
                            ui.button('emergency stop', on_click=lambda: self.system.field_friend.estop.set_soft_estop(True)).props('color=red') \
                                .classes('py-3 px-6 text-lg').bind_visibility_from(self.system.field_friend.estop, 'is_soft_estop_active', value=False)
                            ui.button('emergency reset', on_click=lambda: self.system.field_friend.estop.set_soft_estop(False)) \
                                .props('color=red-700 outline').classes('py-3 px-6 text-lg') \
                                .bind_visibility_from(self.system.field_friend.estop, 'is_soft_estop_active', value=True)
                        ui.space()
                        with ui.row():
                            rosys.automation.automation_controls(self.system.automator)
