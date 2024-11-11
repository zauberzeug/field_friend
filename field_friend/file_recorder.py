import json
import logging
import os
import shutil
from datetime import datetime

import cv2
import numpy as np
import rosys
from nicegui import ui


class FileRecorder:
    """Class to handle recording of sensor data."""

    def __init__(self, directory: str = '/home/zauberzeug/recordings'):
        self.log = logging.getLogger('recorder')
        self.start_time = 0.0

        self.directory = directory
        self.recording_name: str | None = None
        rosys.on_shutdown(self.stop)

    async def start(self, recording_name: str | None = None):
        """Start the recording."""
        self.recording_name = datetime.now().strftime('%Y-%m-%d_%H-%M-%S') if recording_name is None else recording_name
        self.start_time = rosys.time()

    async def stop(self):
        """Stop the recording and zip the directory."""
        if not self.recording_name:
            return
        recording_path = os.path.join(self.directory, self.recording_name)

        def do_zip():
            shutil.make_archive(recording_path, 'zip', recording_path)
            shutil.rmtree(recording_path)
        await rosys.run.io_bound(do_zip)
        rosys.notify(f'Successfully recorded {self.recording_name}', 'positive')
        self.recording_name = None

    @property
    def is_recording(self):
        return self.recording_name is not None

    async def write_json(self, topic: str, timestamp: float, data: dict):
        """Write dictionary data to a JSON file."""
        if not self.is_recording:
            return
        assert isinstance(self.recording_name, str)
        topic_dir = os.path.join(self.directory, self.recording_name, topic)
        os.makedirs(topic_dir, exist_ok=True)
        filename = str(timestamp).replace('.', '_') + '.json'
        filepath = os.path.join(topic_dir, filename)

        def write_to_file():
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=4)
        await rosys.run.io_bound(write_to_file)

    async def write_image(self, topic: str, timestamp: float, data: np.ndarray):
        """Write an image to a JPG file."""
        if not self.is_recording:
            return
        assert isinstance(self.recording_name, str)
        topic_dir = os.path.join(self.directory, self.recording_name, topic)
        os.makedirs(topic_dir, exist_ok=True)
        filename = str(timestamp).replace('.', '_') + '.jpg'
        filepath = os.path.join(topic_dir, filename)

        def write_to_file():
            cv2.imwrite(filepath, data)
        await rosys.run.io_bound(write_to_file)

    def recorder_ui(self):
        ui.label('Recorder').classes('w-full text-center text-bold')
        ui.separator()
        with ui.row():
            ui.button('start', on_click=self.start)
            ui.button('stop', on_click=self.stop)
        with ui.row():
            with ui.column().bind_visibility_from(self, 'is_recording'):
                ui.label('Recording:')
                ui.label('').bind_text_from(self, 'recording_name', lambda recording_name: recording_name)
                ui.label('').bind_text_from(self, 'start_time', lambda start_time: f'{rosys.time() - start_time:.2f}s')

    def __enter__(self):
        """Context manager entry point."""
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit point."""
        await self.stop()
