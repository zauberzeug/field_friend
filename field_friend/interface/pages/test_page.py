from typing import TYPE_CHECKING

from nicegui import ui

from ..components import header_bar, status_drawer, system_bar, test

if TYPE_CHECKING:
    from field_friend.system import System


class test_page():

    def __init__(self, system: 'System') -> None:

        @ui.page('/test')
        def test_page():
            drawer = status_drawer(system, system.field_friend, system.gnss, system.odometer)
            header_bar(system, drawer)
            system_bar()
            with ui.column().classes('w-full items-stretch'):
                with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                    test(system.field_friend, system.steerer, system.odometer, system.automator, system.driver,
                         system.usb_camera_provider)
