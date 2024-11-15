from nicegui import ui

from ..system import System
from .components import Monitoring as monitoring
from .components import create_header


class MonitorPage:

    def __init__(self, system: System, dev: bool = False) -> None:
        self.system = system
        self.dev = dev

        @ui.page('/monitor')
        def page() -> None:
            create_header(system)
            self.content()

    def content(self) -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        monitoring(self.system)
