import inspect
from typing import Awaitable, Callable, Optional, Union

from nicegui import ui
from rosys import config
from rosys.automation import Automator


class automation_controls:
    """This UI element contains start/stop/pause/resume buttons for controlling a given automator.

    See [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) for a simple example of the automation controls.
    """

    def __init__(
            self, automator: Automator, *, can_start: Optional[Callable[[],
                                                                        Union[bool, Awaitable[bool]]]] = None,) -> None:
        async def start() -> None:
            if inspect.iscoroutinefunction(can_start):
                if not await can_start():
                    return
            elif can_start is not None and not can_start():
                return
            automator.start()
        self.play_button = ui.button(on_click=start) \
            .props('icon=play_arrow unelevated').tooltip('start automation').classes('py-3 px-6 text-lg')
        self.pause_button = ui.button(on_click=lambda: automator.pause(because='pause button was pressed')) \
            .props('icon=pause outline').tooltip('pause automation').classes('py-3 px-6 text-lg')

        async def resume() -> None:
            automator.resume()
        self.resume_button = ui.button(on_click=resume) \
            .props('icon=play_arrow outline').tooltip('resume automation').classes('py-3 px-6 text-lg')

        async def stop() -> None:
            automator.stop(because='stop button was pressed')
        self.stop_button = ui.button(on_click=stop) \
            .props('icon=stop outline').tooltip('stop automation').classes('py-3 px-6 text-lg')

        def refresh() -> None:
            self.play_button.visible = automator.is_stopped
            self.pause_button.visible = automator.is_running
            self.resume_button.visible = automator.is_paused
            self._disable(self.play_button, automator.default_automation is None or not automator.enabled)
            self._disable(self.stop_button, automator.is_stopped)

        ui.timer(config.ui_update_interval, refresh)

    def _disable(self, button: ui.button, should_disable: bool) -> None:
        if should_disable:
            button.props('disable')
        else:
            button.props(remove='disable')
