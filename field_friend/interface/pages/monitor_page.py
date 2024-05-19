from typing import TYPE_CHECKING

from nicegui import ui

from ..components import monitoring

if TYPE_CHECKING:
    from field_friend.system import System


class monitor_page():

    def __init__(self, page_wrapper, system: 'System', dev: bool = False) -> None:
        self.system = system
        self.dev = dev

        @ui.page('/monitor')
        def page() -> None:
            page_wrapper()
            self.content()

    def content(self) -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        monitoring(self.system.usb_camera_provider,
                   self.system.mjpeg_camera_provider,
                   self.system.detector,
                   self.system.monitoring_detector,
                   self.system.plant_locator,
                   self.system.automator,
                   self.system.field_friend,
                   self.system,
                   )
