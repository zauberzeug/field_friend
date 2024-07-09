from typing import TYPE_CHECKING

from nicegui import ui

if TYPE_CHECKING:
    from field_friend.system import System


class io_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/io')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        with ui.row().style(f'width: calc(100vw - 2rem); flex-wrap: nowrap;'):
            ui.label('IO Page')
