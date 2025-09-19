from __future__ import annotations

from typing import TYPE_CHECKING

from rosys.analysis import videos_page

from .components import create_header

if TYPE_CHECKING:
    from ..system import System


class VideosPage(videos_page):
    def __init__(self, system: System) -> None:
        super().__init__()
        self.system = system

    async def _content(self) -> None:
        create_header(self.system)
        await super()._content()

    def _video_page_content(self, name: str) -> None:
        create_header(self.system)
        super()._video_page_content(name)
