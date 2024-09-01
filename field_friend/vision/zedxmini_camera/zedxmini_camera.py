import abc
import base64
import logging
from dataclasses import dataclass

import numpy as np
import requests
from rosys.vision import CalibratableCamera, Image, ImageSize


@dataclass
class StereoFrame:
    timestamp: float
    left: Image
    right: Image
    depth: Image
    depth_map: np.ndarray


class StereoCamera(CalibratableCamera, abc.ABC):
    @abc.abstractmethod
    def get_depth(self, x, y, size=0) -> float | None:
        pass


class ZedxminiCamera(StereoCamera):
    def __init__(self, camera_id='zedxmini-todo', ip: str = '127.0.0.1', port: int = 8003) -> None:
        self.MAX_IMAGES = 10
        super().__init__(id=camera_id, name='Zedxmini', polling_interval=0.1)
        self.ip = ip
        self.port = port
        self.connected: bool = False
        self.log = logging.getLogger(self.name)
        self.log.setLevel(logging.DEBUG)
        # TODO: get calibration
        self.set_perfect_calibration(width=1920, height=1200)
        # rosys.on_startup(self.setup_camera)

    async def connect(self) -> None:
        await super().connect()
        url = f'http://{self.ip}:{self.port}/information'
        response = requests.get(url, timeout=2.0)
        if response.status_code != 200:
            self.log.warning(f"response.status_code: {response.status_code}")
            self.connected = False
            return
        data = response.json()
        print(data)
        self.connected = True

    async def capture_image(self) -> None:
        if not self.connected:
            return
        url = f'http://{self.ip}:{self.port}/image'
        response = requests.get(url, timeout=2.0)
        if response.status_code != 200:
            self.log.warning(f"response.status_code: {response.status_code}")
            return
        data = response.json()
        assert 'image' in data
        image_bytes = base64.b64decode(data['image'])

        image = Image(
            camera_id=data['camera_id'],
            size=ImageSize(width=data['width'], height=data['height']),
            time=data['time'],
            data=image_bytes,
            is_broken=data['is_broken'],
            tags=set(data['tags']),
        )
        self._add_image(image)

    def get_depth(self, x, y, size=0) -> float | None:
        url = f'http://{self.ip}:{self.port}/depth?x={x}&y={y}&size={size}'
        response = requests.get(url, timeout=2.0)
        if response.status_code != 200:
            self.log.warning(f"response.status_code: {response.status_code}")
            return None
        return float(response.text)

    @property
    def is_connected(self) -> bool:
        # TODO: check it in capture_image
        return self.connected
