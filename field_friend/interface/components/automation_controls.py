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
            if callable(can_start):
                # Directly handle both synchronous and asynchronous can_start
                proceed = can_start() if not isinstance(can_start(), Awaitable) else await can_start()
                if not proceed:
                    return
            automator.start()
        play_button = ui.button(on_click=start) \
            .props('icon=play_arrow unelevated').tooltip('start automation')
        pause_button = ui.button(on_click=lambda: automator.pause(because='pause button was pressed')) \
            .props('icon=pause outline').tooltip('pause automation')
        resume_button = ui.button(on_click=automator.resume) \
            .props('icon=play_arrow outline').tooltip('resume automation')
        stop_button = ui.button(on_click=lambda: automator.stop(because='stop button was pressed')) \
            .props('icon=stop outline').tooltip('stop automation')

        def refresh() -> None:
            play_button.visible = automator.is_stopped
            pause_button.visible = automator.is_running
            resume_button.visible = automator.is_paused
            play_button.enabled = automator.default_automation is not None and automator.enabled
            resume_button.enabled = automator.enabled
            stop_button.enabled = not automator.is_stopped

        ui.timer(config.ui_update_interval, refresh)
