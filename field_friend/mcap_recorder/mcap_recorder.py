import logging
import os
from datetime import datetime
from typing import Any, Dict, Optional

import numpy as np
import rosys
from mcap_ros2.writer import Writer as McapWriter
from nicegui import ui

from .schema import (
    GNSS_SCHEMA_NAME,
    GNSS_SCHEMA_TEXT,
    IMAGE_SCHEMA_NAME,
    IMAGE_SCHEMA_TEXT,
    LASER_SCHEMA_NAME,
    LASER_SCHEMA_TEXT,
)


class McapRecorder:
    """Class to handle recording of sensor data to MCAP format."""
    BASE_PATH = '/home/zauberzeug/recordings'

    def __init__(self):
        """Initialize the MCAP recorder.

        Args:
            output_path: Path where the MCAP file will be saved
        """
        self.log = logging.getLogger('mcap_recorder')
        self.writer = None
        self.file_handle = None
        self.schemas = {}
        self.sequence = 0
        self.is_recording = False
        self.filename = ''
        self.start_time = 0.0
        rosys.on_shutdown(self.stop)

    async def start(self, output_path: str | None = None):
        """Initialize the MCAP writer and register schemas."""
        if self.writer and (not self.writer._finished or not self.file_handle.closed):
            self.log.warning('Already recording, stopping first')
            await self.stop()
            return
        if output_path is None:
            formatted_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self.filename = f'{formatted_time}.mcap'
            output_path = os.path.join(self.BASE_PATH, self.filename)
        else:
            self.filename = os.path.basename(output_path)
        self.start_time = rosys.time()
        self.file_handle = open(output_path, 'wb')
        self.writer = McapWriter(self.file_handle, chunk_size=1024*1024)  # 1MB chunks
        # Register all schemas
        self.schemas['image'] = self.writer.register_msgdef(IMAGE_SCHEMA_NAME, IMAGE_SCHEMA_TEXT)
        self.schemas['laser'] = self.writer.register_msgdef(LASER_SCHEMA_NAME, LASER_SCHEMA_TEXT)
        self.schemas['gnss'] = self.writer.register_msgdef(GNSS_SCHEMA_NAME, GNSS_SCHEMA_TEXT)
        self.is_recording = True

    async def stop(self):
        """Close the MCAP file."""
        self.is_recording = False
        await rosys.sleep(0.5)
        if self.writer:
            await rosys.run.io_bound(self.writer.finish)
            # self.writer.finish()
            self.file_handle.close()

    def _get_timestamp(self, timestamp: float) -> dict[str, int]:
        floored_seconds = int(np.floor(timestamp))
        time_dict = {
            'sec': floored_seconds,
            'nanosec': int((timestamp - floored_seconds) * 1e9)
        }
        return time_dict

    async def write_image(self, image_data: np.ndarray, timestamp: float,
                          frame_id: str = "camera", encoding: str = "rgb8"):
        """Write an image message to the MCAP file.

        Args:
            image_data: numpy array containing the image
            timestamp: time in seconds
            frame_id: frame ID for the image
            encoding: image encoding (e.g., "rgb8", "bgr8", "mono8")
        """
        height, width = image_data.shape[:2]
        channels = 1 if len(image_data.shape) == 2 else image_data.shape[2]
        step = width * channels

        message = {
            'header': {
                'stamp': self._get_timestamp(timestamp),
                'frame_id': frame_id,
            },
            'height': height,
            'width': width,
            'encoding': encoding,
            'is_bigendian': 0,
            'step': step,
            'data': image_data.tobytes()
        }

        await self._write_message('/image_raw', self.schemas['image'], message, timestamp)

    async def write_laser_scan(self, ranges: list, intensities: Optional[list] = None,
                               timestamp: float = 0.0, frame_id: str = "laser"):
        """Write a laser scan message to the MCAP file.

        Args:
            ranges: array of range measurements
            intensities: optional array of intensity measurements
            timestamp: time in seconds
            frame_id: frame ID for the laser scan
        """
        if intensities is None:
            intensities = np.zeros_like(ranges, dtype=np.float32).tolist()

        message = {
            'header': {
                'stamp': self._get_timestamp(timestamp),
                'frame_id': frame_id,
            },
            'angle_min': -np.pi,
            'angle_max': np.pi,
            'angle_increment': 2 * np.pi / len(ranges),
            'time_increment': 0.0,
            'scan_time': 0.1,
            'range_min': 0.0,
            'range_max': 30.0,
            'ranges': ranges,
            'intensities': intensities
        }

        await self._write_message('/scan', self.schemas['laser'], message, timestamp)

    async def write_gnss(self, latitude: float, longitude: float, altitude: float, latitude_std_dev: float, longitude_std_dev: float,
                         timestamp: float = 0.0, frame_id: str = "gps"):
        """Write a GNSS message to the MCAP file.

        Args:
            latitude: latitude in degrees
            longitude: longitude in degrees
            altitude: altitude in meters
            latitude_std_dev: latitude standard deviation in degrees
            longitude_std_dev: longitude standard deviation in degrees
            timestamp: time in seconds
            frame_id: frame ID for the GNSS
        """
        message = {
            'header': {
                'stamp': self._get_timestamp(timestamp),
                'frame_id': frame_id,
            },
            'status': {
                'status': 2,  # 2=SBAS
                'service': 1  # 1=GPS
            },
            'latitude': latitude,
            'longitude': longitude,
            'altitude': altitude,
            'position_covariance': [latitude_std_dev**2.0, 0, 0, 0, longitude_std_dev**2.0, 0, 0, 0, 0],
            'position_covariance_type': 2
        }

        await self._write_message('/fix', self.schemas['gnss'], message, timestamp)

    async def _write_message(self, topic: str, schema: Dict, message: Dict[str, Any], timestamp: float):
        """Helper method to write a message to the MCAP file."""
        def write():
            self.writer.write_message(
                topic=topic,
                schema=schema,
                message=message,
                log_time=int(timestamp * 1e9),  # Convert to nanoseconds
                publish_time=int(timestamp * 1e9),
                sequence=self.sequence
            )
            self.sequence += 1
        await rosys.run.io_bound(write)

    def recorder_ui(self):
        ui.label('Recorder').classes('w-full text-center text-bold')
        ui.separator()
        with ui.row():
            ui.button('start', on_click=self.start)
            ui.button('stop', on_click=self.stop)
        with ui.row():
            with ui.column().bind_visibility_from(self, 'is_recording'):
                ui.label('').bind_text_from(self, 'start_time', lambda start_time: f'Recording \
                    {self.filename} for {rosys.time() - start_time:.2f}s aaaaaaaaaaaaaaaaaaaaa')

    def __enter__(self):
        """Context manager entry point."""
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit point."""
        await self.stop()

    # def __exit__(self, exc_type, exc_val, exc_tb):
    #     """Context manager exit point."""
    #     awaitself.close()
