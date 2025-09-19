from __future__ import annotations

from typing import TYPE_CHECKING

from rosys.analysis import logging_page

from .components import create_header

if TYPE_CHECKING:
    from ..system import System


class LoggingPage(logging_page):
    def __init__(self, system: System, group_names: list[str]) -> None:
        super().__init__(group_names)
        self.system = system

    def _content(self) -> None:
        create_header(self.system)
        super()._content()
