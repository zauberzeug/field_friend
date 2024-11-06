import logging
import uuid

import rosys
from nicegui import ui

from ...automations import Path, PathProvider, PathRecorder


class PathPlanner:

    def __init__(
            self, path_provider: PathProvider, path_recorder: PathRecorder, automator: rosys.automation.Automator, ) -> None:
        self.path_provider = path_provider
        self.path_recorder = path_recorder
        self.automator = automator
        self.log = logging.getLogger('field_friend.path_planner')
        self.new_name = ''
        with ui.card():
            with ui.row():
                ui.button('Add path', on_click=self.add_path)
                ui.button('Clear paths', on_click=self.clear_paths)
            with ui.row():
                self.show_path_settings()  # type: ignore
        # rosys.on_repeat(self.check_for_reference, 1)

    @ui.refreshable
    def show_path_settings(self) -> None:
        for path in self.path_provider.paths:
            with ui.card().classes('items-stretch'):
                with ui.row().classes('items-center'):
                    ui.icon('route').props('size=sm color=primary')
                    ui.input('name', value=f'{path.name}').bind_value(path, 'name')
                    ui.button(on_click=lambda path=path: self.delete_path(path), icon='delete') \
                        .props('color=warning fab-mini flat').classes('ml-auto')

                with ui.row().classes('items-canter'):
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=path.name: p != name):
                        ui.button('record', on_click=lambda path=path: self.automator.start(self.path_recorder.record_path(path))) \
                            .props('icon=radio_button_checked color=grey fab-mini flat') \
                            .tooltip('start recording')
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_recording', lambda p, name=path.name: p == name):
                        ui.button('recording...', on_click=self.stop_recording, icon='radio_button_checked', color='red') \
                            .bind_visibility_from(self.path_recorder, 'state', lambda s: s == 'recording') \
                            .props('fab-mini flat') \
                            .tooltip('stop recording')
                    with ui.row().bind_visibility_from(path, 'path_segments', lambda path_segments: path_segments != []):
                        ui.button('visualize', icon='visibility',
                                  on_click=lambda _, path=path: self.path_provider.SHOW_PATH.emit(path.path_segments)) \
                            .props('fab-mini flat') \
                            .tooltip('visualize path')
                        ui.button('drive', icon='play_arrow',
                                  on_click=lambda path=path: self.automator.start(self.path_recorder.drive_path(path))) \
                            .bind_visibility_from(self.path_recorder, 'state', lambda s: s == 'idle') \
                            .props('fab-mini flat') \
                            .tooltip('drive recorded path')
                    with ui.row().bind_visibility_from(self.path_recorder, 'current_path_driving', lambda p, name=path.name: p == name):
                        ui.button('stop', on_click=self.stop_driving, icon='stop') \
                            .bind_visibility_from(self.path_recorder, 'state', lambda s: s == 'driving') \
                            .props('fab-mini flat') \
                            .tooltip('stop driving')

    def stop_recording(self) -> None:
        self.path_recorder.state = 'idle'
        self.path_recorder.current_path_recording = ''
        self.show_path_settings.refresh()

    def stop_driving(self) -> None:
        self.automator.stop('Stopped because stop button was pressed')
        self.path_recorder.state = 'idle'
        self.show_path_settings.refresh()

    def add_path(self) -> None:
        path = Path(name=f'{uuid.uuid4()!s}')
        self.path_provider.add_path(path)
        self.show_path_settings.refresh()

    def delete_path(self, path: Path) -> None:
        self.path_provider.remove_path(path)
        self.show_path_settings.refresh()

    def clear_paths(self) -> None:
        self.path_provider.clear_paths()
        self.show_path_settings.refresh()
