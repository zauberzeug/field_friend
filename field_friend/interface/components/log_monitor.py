from collections import deque
from datetime import datetime

import rosys
from nicegui import ui
from rosys.event import Event


class LogMonitor(rosys.persistence.Persistable):
    MAX_LINES = 100

    def __init__(self, max_lines: int = MAX_LINES) -> None:
        super().__init__()
        self.max_lines = max_lines
        self.lines: deque[str] = deque([], max_lines)

        self.NEW_LINE: Event[str] = Event()
        """a new line was added to the log (argument: line)"""

        rosys.NEW_NOTIFICATION.register(self.handle_notification)

    def handle_notification(self, message: str) -> None:
        line = f'{datetime.now():%m/%d/%Y %H:%M:%S} {message}'
        self.lines.append(line)
        self.NEW_LINE.emit(line)
        self.request_backup()

    def backup_to_dict(self) -> dict:
        return {
            'logs': list(self.lines),
            'max_lines': self.max_lines,
        }

    def restore_from_dict(self, data: dict) -> None:
        logs = data.get('logs', [])
        self.max_lines = data.get('max_lines', self.MAX_LINES)
        self.lines = deque(logs, self.max_lines)

    def ui(self) -> None:
        ui.label('Log Monitor').classes('text-center text-bold')
        with ui.log(max_lines=self.max_lines).classes('text-xs') as log:
            log.push('\n'.join(self.lines))
            self.NEW_LINE.register_ui(log.push)
            ui.run_javascript(f'getElement({log.id}).scrollTop = getElement({log.id}).scrollHeight')
