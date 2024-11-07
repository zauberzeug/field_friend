from nicegui import ui

from ..system import System
from .components import DriveTest as drive_test
from .components import create_header


class TestPage:

    def __init__(self, system: System) -> None:
        self.system = system

        @ui.page('/test')
        def page() -> None:
            create_header(system)
            self.content()

    def content(self):
        with ui.column().classes('w-full items-stretch'):
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                drive_test(field_friend=self.system.field_friend,
                           steerer=self.system.steerer,
                           odometer=self.system.odometer,
                           automator=self.system.automator,
                           driver=self.system.driver,
                           camera_provider=self.system.camera_provider)
