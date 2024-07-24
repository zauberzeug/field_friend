from typing import TYPE_CHECKING

from nicegui import ui

from ..components import development
from .main_page import main_page

if TYPE_CHECKING:
    from field_friend.system import System


class dev_page():

    def __init__(self, page_wrapper, system: 'System') -> None:
        self.system = system

        @ui.page('/dev')
        def page() -> None:
            page_wrapper()
            # main_pg = main_page(page_wrapper, system)
            # main_pg.content(devmode=True)
            self.content()

    def content(self) -> None:
        development(self.system)
