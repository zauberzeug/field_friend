import logging
from typing import TYPE_CHECKING, Awaitable, Callable, Optional, Union

from nicegui import ui
from rosys import config

if TYPE_CHECKING:
    from field_friend.system import System


class automation_controls:
    """This UI element contains start/stop/pause/resume buttons for controlling a given automator.

    See [Play-pause-stop](https://rosys.io/examples/play-pause-stop/) for a simple example of the automation controls.
    """

    def __init__(self, system: 'System') -> None:
        async def start() -> None:
            if not await self.can_start():
                return
            system.automator.start()

        self.log = logging.getLogger('field_friend.automation_controls')
        self.system = system
        play_button = ui.button(on_click=start) \
            .props('icon=play_arrow unelevated').tooltip('start automation')
        pause_button = ui.button(on_click=lambda: system.automator.pause(because='pause button was pressed')) \
            .props('icon=pause outline').tooltip('pause automation')
        resume_button = ui.button(on_click=system.automator.resume) \
            .props('icon=play_arrow outline').tooltip('resume automation')
        stop_button = ui.button(on_click=lambda: system.automator.stop(because='stop button was pressed')) \
            .props('icon=stop outline').tooltip('stop automation')

        with ui.dialog() as self.dialog, ui.card():
            self.dialog_label = ui.label('Do you want to continue the canceled automation').classes('text-lg')
            with ui.row():
                ui.button('Yes', on_click=lambda: self.dialog.submit('Yes'))
                ui.button('No', on_click=lambda: self.dialog.submit('No'))
                ui.button('Cancel', on_click=lambda: self.dialog.submit('Cancel'))

        def refresh() -> None:
            play_button.visible = system.automator.is_stopped
            pause_button.visible = system.automator.is_running
            resume_button.visible = system.automator.is_paused
            play_button.enabled = system.automator.default_automation is not None and system.automator.enabled
            resume_button.enabled = system.automator.enabled
            stop_button.enabled = not system.automator.is_stopped

        ui.timer(config.ui_update_interval, refresh)

    async def can_start(self) -> bool:
        return True

    async def can_mowing_start(self) -> bool:
        self.log.info('Checking mowing automation')
        if self.system.mowing.current_path_segment is None:
            self.system.mowing.continue_mowing = False
            return True
        self.dialog_label.text = 'Do you want to continue the canceled mowing automation?'
        result = await self.dialog
        if result == 'Yes':
            self.system.mowing.continue_mowing = True
        elif result == 'No':
            self.system.mowing.continue_mowing = False
        elif result == 'Cancel':
            return False
        return True

    async def can_weeding_start(self) -> bool:
        self.log.info('Checking weeding automation')
        if not self.system.weeding.current_row or not self.system.weeding.current_segment:
            self.system.weeding.continue_canceled_weeding = False
            return True
        self.dialog_label.text = f'''Do you want to continue the canceled weeding automation on row {
            self.system.weeding.current_row.name}?'''
        result = await self.dialog
        if result == 'Yes':
            self.system.weeding.continue_canceled_weeding = True
        elif result == 'No':
            self.system.weeding.continue_canceled_weeding = False
        elif result == 'Cancel':
            return False
        return True
