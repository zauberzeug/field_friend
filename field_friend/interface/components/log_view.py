from collections import deque
from datetime import datetime

import rosys
from nicegui import ui


class LogMonitor(rosys.persistence.PersistentModule):
    MAX_LINES = 100

    def __init__(self) -> None:
        super().__init__()
        self.lines: deque[str] = deque([], self.MAX_LINES)

        self.NEW_LINE = rosys.event.Event()
        """a new line was added to the log (argument: line)"""

        def handle_notification(message: str) -> None:
            line = f'{datetime.now():%m/%d/%Y %H:%M:%S} {message}'
            self.lines.append(line)
            self.NEW_LINE.emit(line)
            self.request_backup()
        rosys.NEW_NOTIFICATION.register(handle_notification)

    def backup(self) -> dict:
        return {'logs': list(self.lines)}

    def restore(self, data: dict) -> None:
        self.lines = deque(data.get('logs', []), self.MAX_LINES)


class LogView(ui.log):

    def __init__(self) -> None:
        log_monitor = LogMonitor()
        super().__init__(max_lines=log_monitor.MAX_LINES)
        self.push('\n'.join(log_monitor.lines))
        self.classes('text-xs')
        log_monitor.NEW_LINE.register_ui(self.push)
        ui.run_javascript(f'getElement({self.id}).scrollTop = getElement({self.id}).scrollHeight')
