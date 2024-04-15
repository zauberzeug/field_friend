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
        # ui.query('body').classes('bg-black text-white')
        # ui.query('.nicegui-content').classes('p-0 h-screen')
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        if self.system.is_real:
            monitoring(self.system.usb_camera_provider, self.system.mjpeg_camera_provider,
                       self.system.plant_locator, self.system.person_locator, self.system.automator, self.system.field_friend, self.system)
