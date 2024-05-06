from typing import TYPE_CHECKING, Awaitable, Callable, Optional, Union

from nicegui import ui
from rosys import config
from rosys.automation import Automator

from ...automations import KpiProvider

if TYPE_CHECKING:
    from field_friend.system import System


class automation_controls:
    """This UI element contains start/stop/pause/resume buttons for controlling a given automator.

    See [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) for a simple example of the automation controls.
    """

    def __init__(
            self, system: 'System', *, can_start: Optional[Callable[[], Union[bool, Awaitable[bool]]]] = None,) -> None:
        async def start() -> None:
            if callable(can_start):
                # Directly handle both synchronous and asynchronous can_start
                result = can_start()
                proceed = result if not isinstance(result, Awaitable) else await result
                if not proceed:
                    return

            current_automation = next(key for key, value in system.automations.items()
                                      if value == system.automator.default_automation)
            if current_automation == 'weeding' or current_automation == 'monitoring' or current_automation == 'collecting (demo)':
                if system.weeding.continue_canceled_weeding is not True:
                    system.kpi_provider.clear_weeding_kpis()
            elif current_automation == 'mowing':
                if system.mowing.continue_mowing is not True:
                    system.kpi_provider.clear_mowing_kpis()
            system.automator.start()
        play_button = ui.button(on_click=start) \
            .props('icon=play_arrow unelevated').tooltip('start automation')
        pause_button = ui.button(on_click=lambda: system.automator.pause(because='pause button was pressed')) \
            .props('icon=pause outline').tooltip('pause automation')
        resume_button = ui.button(on_click=system.automator.resume) \
            .props('icon=play_arrow outline').tooltip('resume automation')
        stop_button = ui.button(on_click=lambda: system.automator.stop(because='stop button was pressed')) \
            .props('icon=stop outline').tooltip('stop automation')

        def refresh() -> None:
            play_button.visible = system.automator.is_stopped
            pause_button.visible = system.automator.is_running
            resume_button.visible = system.automator.is_paused
            play_button.enabled = system.automator.default_automation is not None and system.automator.enabled
            resume_button.enabled = system.automator.enabled
            stop_button.enabled = not system.automator.is_stopped

        ui.timer(config.ui_update_interval, refresh)
