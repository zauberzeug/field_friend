from nicegui import ui

from ..system import System
from .components import create_header, monitoring


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
        monitoring(self.system.camera_provider,
                   self.system.mjpeg_camera_provider,
                   self.system.detector,
                   self.system.monitoring_detector,
                   self.system.plant_locator,
                   self.system.automator,
                   self.system.field_friend,
                   self.system,
                   )
