from datetime import timedelta

import psutil
import rosys
from nicegui import ui


def SystemBar() -> None:
    with ui.footer():
        timer = ui.label().classes('flex-grow')
        cpu_label = ui.label().classes('flex-grow')

        async def update_status() -> None:
            timer.set_text(f'{timedelta(seconds=rosys.uptime())}')
            cpu_label.set_text(f'CPU: {psutil.cpu_percent():.0f}%')
        ui.timer(rosys.config.ui_update_interval, update_status)
