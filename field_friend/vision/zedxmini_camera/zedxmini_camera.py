import logging
from dataclasses import dataclass

import numpy as np
import rosys
from rosys.vision import CalibratableCamera, Image


@dataclass
class StereoFrame:
    timestamp: float
    left: Image
    right: Image
    depth: Image
    depth_map: np.ndarray


class ZedxminiCamera(CalibratableCamera):
    def __init__(self, camera_id='zedxmini-todo', ip: str = '192.168.178.27', port: int = 8003) -> None:
        super().__init__(id=camera_id, name='Zedxmini')
        self.ip = ip
        self.port = port
        self.log = logging.getLogger(self.name)
        self.log.setLevel(logging.DEBUG)
        self.set_perfect_calibration(width=1920, height=1200)
    #     rosys.on_repeat(self.get_image, 1.0/10.0)

    # async def get_image(self) -> None:

    #     self._add_image(left_image)

    async def capture_image(self) -> None:
        # TODO
        pass

    def get_latest_image_url(self) -> str:
        return f'http://{self.ip}:{self.port}/zed/left?{rosys.time()}'

    def get_depth(self, x, y, size=0, lense_distance_in_mm=7):
        # TODO
        pass

    @property
    def is_connected(self) -> bool:
        return True
