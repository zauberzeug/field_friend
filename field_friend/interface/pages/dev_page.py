from typing import TYPE_CHECKING

from nicegui import ui

from ..components import development

if TYPE_CHECKING:
    from field_friend.system import System


class dev_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/dev')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        development(self.system)
