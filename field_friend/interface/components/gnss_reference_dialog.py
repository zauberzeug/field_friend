from nicegui import ui
from rosys.hardware import Gnss


class GnssReferenceDialog(ui.dialog):
    def __init__(self, gnss: Gnss):
        super().__init__()
        self.gnss = gnss
        with self, ui.card():
            ui.label('The reference is to far away from the current position which would lead to issues in the navigation. Do you want to set it now?')
            with ui.row():
                ui.button('Update reference', on_click=self.update_reference).props('outline color=warning') \
                    .tooltip('Set current position as geo reference and restart the system').classes('ml-auto').style('display: block; margin-top:auto; margin-bottom: auto;')
                ui.button('Cancel', on_click=self.close)

    def update_reference(self):
        self.gnss.update_reference()
        self.close()
