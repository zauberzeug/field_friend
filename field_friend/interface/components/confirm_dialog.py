from nicegui import ui


class ConfirmDialog(ui.dialog):
    def __init__(self, text: str = 'Are you sure?', delay: float = 3.0) -> None:
        super().__init__()
        self.delay = delay
        with self, ui.card():
            ui.label(text)
            with ui.row():
                yes_button = ui.button('Yes', on_click=lambda: self.submit(True))
                yes_button.disable()
                ui.button('No', on_click=lambda: self.submit(False))
        ui.timer(delay, callback=yes_button.enable, once=True)
