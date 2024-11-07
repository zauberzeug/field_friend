from nicegui import ui

from ..system import System
from .components import create_development_ui, create_header


class DevPage:

    def __init__(self, system: System) -> None:
        self.system = system

        @ui.page('/dev')
        def page() -> None:
            create_header(system)
            create_development_ui(system)
