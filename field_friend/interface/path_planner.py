import logging
import uuid

import rosys
from nicegui import ui

from ..automations import PathRecorder
from ..navigation import Gnss


class path_planner:

    def __init__(
            self, path_recorder: PathRecorder, automator: rosys.automation.Automator, driver: rosys.driving.Driver,
            gnss: Gnss) -> None:
        self.path_recorder = path_recorder
        self.automator = automator
        self.driver = driver
        self.gnss = gnss
        self.log = logging.getLogger('field_friend.path_planner')
        self.new_name = ''
        with ui.card():
            with ui.row():
                ui.button('Add path', on_click=self.add_path)
                ui.button('Clear paths', on_click=self.clear_paths)
            with ui.row():
                self.show_path_settings()
        # rosys.on_repeat(self.check_for_reference, 1)

    @ ui.refreshable
    def show_path_settings(self) -> None:
        for name, path in self.path_recorder.paths.items():
            with ui.card().classes('items-stretch'):
                with ui.row():
                    def change_name(name: str):
                        self.new_name = name
                    ui.input('name', value=f'{name}', on_change=lambda e: change_name(e.value)).on(
                        'blur', lambda name=name: self.update_path_name(self.new_name, name))
                    ui.button(on_click=lambda name=name: self.delete_path(name)).props(
                        'icon=delete color=warning fab-mini flat').classes('ml-auto')

                with ui.row().classes('items-canter'):
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=name: p != name):
                        ui.button('record', on_click=lambda name=name: self.automator.start(
                            self.path_recorder.record_path(name))).props(
                            'icon=radio_button_checked color=grey fab-mini flat').tooltip('start recording')
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=name: p == name):
                        ui.button(
                            'recording...', on_click=self.stop_recording).props(
                            'icon=radio_button_checked color=red fab-mini flat').bind_visibility_from(
                            self.path_recorder, 'state', lambda s: s == 'recording').tooltip('stop recording')
                    with ui.row().bind_visibility_from(
                            self.path_recorder, 'paths', lambda p, name=name: p[name] != []):
                        ui.button('play', on_click=lambda name=name: self.automator.start(
                            self.path_recorder.drive_path(name))).props(
                            'icon=play_arrow fab-mini flat').tooltip('drive recorded path').bind_visibility_from(self.path_recorder,
                                                                                                                 'state', lambda s: s == 'idle')
                        ui.button('stop', on_click=self.stop_driving).props(
                            'icon=stop fab-mini flat').tooltip('stop driving').bind_visibility_from(self.path_recorder,
                                                                                                    'state', lambda s: s == 'driving')

    def stop_recording(self) -> None:
        self.path_recorder.state = 'idle'
        self.path_recorder.current_path_recording = ''
        self.show_path_settings.refresh()

    def stop_driving(self) -> None:
        self.automator.stop('Stopped because stop button was pressed')
        self.path_recorder.state = 'idle'
        self.show_path_settings.refresh()

    def update_path_name(self, new_name: str, old_name: str) -> None:
        self.log.info(f'update path name {old_name} -> {new_name}')
        self.path_recorder.paths[new_name] = self.path_recorder.paths[old_name]
        del self.path_recorder.paths[old_name]
        self.new_name = ''
        self.show_path_settings.refresh()
        self.path_recorder.invalidate()

    def add_path(self) -> None:
        # if self.gnss.reference_lat is None or self.gnss.reference_lon is None:
        #     rosys.notify('Couldn\'t add path, no reference position set', 'warning')
        #     return
        name = f'path {str(uuid.uuid4())}'
        self.path_recorder.paths[name] = []
        self.show_path_settings.refresh()
        self.path_recorder.invalidate()

    def delete_path(self, name: str) -> None:
        del self.path_recorder.paths[name]
        self.show_path_settings.refresh()
        self.path_recorder.invalidate()

    def clear_paths(self) -> None:
        self.path_recorder.paths.clear()
        self.show_path_settings.refresh()
        self.path_recorder.invalidate()

    def check_for_reference(self) -> None:
        if self.gnss.reference_lat is None or self.gnss.reference_lon is None:
            self.clear_paths()
