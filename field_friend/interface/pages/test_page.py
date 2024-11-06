from typing import TYPE_CHECKING

from nicegui import ui

from ..components import DriveTest as drive_test

if TYPE_CHECKING:
    from field_friend.system import System


class test_page:

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/test')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self):
        with ui.column().classes('w-full items-stretch'):
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                drive_test(self.system.field_friend, self.system.steerer, self.system.odometer, self.system.automator,
                           self.system.driver, self.system.camera_provider)
