from typing import TYPE_CHECKING

from nicegui import ui

from ..components import camera_card, leaflet_map, operation, robot_scene

if TYPE_CHECKING:
    from field_friend.system import System


class main_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/')
        def page() -> None:
            page_wrapper()
            self.content(devmode=False)

    def content(self, devmode) -> None:
        page_height = '50vh' if devmode else 'calc(100vh - 150px)'
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        with ui.row().style(f'height:{page_height}; width: calc(100vw - 2rem); flex-wrap: nowrap;'):
            with ui.splitter(horizontal=False, reverse=False, value=35, limits=(10, 50)).classes('w-full h-full') as splitter:
                with splitter.before:
                    with ui.column().classes('h-full p-2').style('width: 100%;'):
                        leaflet_map_landing = leaflet_map(self.system, False)
                        leaflet_map_landing.m.classes(
                            'h-full w-full')
                with splitter.after:
                    with ui.row().classes('h-full ml-2 m-2').style('width: calc(100% - 1rem)'):
                        with ui.column().style('width: 55%; height: 100%; flex-wrap: nowrap;'):
                            operation(self.system, leaflet_map_landing)
                        with ui.column().classes('h-full').style('width: calc(45% - 2rem); flex-wrap: nowrap;'):
                            with ui.card().classes('w-full h-full p-0').style('margin-bottom: 10px;'):
                                with ui.scroll_area().classes('w-full h-full'):
                                    with ui.card().classes('w-full'):
                                        camera_card(self.system.usb_camera_provider, self.system.automator, self.system.detector, self.system.plant_locator, self.system.field_friend,
                                                    self.system.puncher)
                                    with ui.card().classes('w-full'):
                                        robot_scene(self.system)
                with splitter.separator:
                    ui.button(icon='drag_indicator').props('round')
