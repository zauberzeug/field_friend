import abc
import logging
from typing import Any, Self

import aiohttp
import rosys
from rosys import persistence
from rosys.geometry import Point3d
from rosys.vision import CalibratableCamera, Calibration, Image, ImageSize, Intrinsics


class StereoCamera(CalibratableCamera, abc.ABC):
    @abc.abstractmethod
    async def get_point(self, x, y) -> Point3d | None:
        pass


class ZedxminiCamera(StereoCamera):
    ip: str = 'localhost'
    port: int = 8003

    def __init__(self, ip: str | None = None, port: int | None = None, focal_length: float = 747.0735473632812, **kwargs) -> None:
        self.MAX_IMAGES = 10
        super().__init__(**kwargs)
        if ip is not None:
            self.ip = ip
        if port is not None:
            self.port = port
        self.focal_length = focal_length
        self.connected: bool = False
        self.log = logging.getLogger(self.name)
        self.log.setLevel(logging.DEBUG)
        self.camera_information: dict[str, Any] = {}
        rosys.on_repeat(self._capture_image, interval=0.1)

    async def connect(self) -> None:
        await super().connect()
        self.connected = await self.setup_camera_information()

    @staticmethod
    async def get_camera_information(ip: str | None = None, port: int | None = None) -> dict[str, Any] | None:
        ip = ZedxminiCamera.ip if ip is None else ip
        port = ZedxminiCamera.port if port is None else port
        url = f'http://{ip}:{port}/information'
        data: dict[str, Any] | None = None
        async with aiohttp.ClientSession() as session:
            try:
                async with session.get(url, timeout=aiohttp.ClientTimeout(2.0)) as response:
                    if response.status != 200:
                        logging.warning(f'response.status: {response.status}')
                        return None
                    data = await response.json()
            except aiohttp.ClientError as e:
                logging.error(f'Error capturing image: {e!s}')
                return None
            except TimeoutError:
                logging.error('Request timed out')
                return None
        if data is None:
            return None
        return data

    async def setup_camera_information(self) -> bool:
        camera_information = await self.get_camera_information(self.ip, self.port)
        if camera_information is None:
            return False
        assert 'calibration' in camera_information
        assert 'left_cam' in camera_information['calibration']
        assert 'fx' in camera_information['calibration']['left_cam']
        self.camera_information = camera_information
        self.focal_length = camera_information['calibration']['left_cam']['fy']
        return True

    async def _capture_image(self) -> None:
        if not self.connected:
            return
        url = f'http://{self.ip}:{self.port}/image'
        data: dict[str, Any] | None = None
        async with aiohttp.ClientSession() as session:
            try:
                async with session.get(url, timeout=aiohttp.ClientTimeout(2.0)) as response:
                    if response.status != 200:
                        self.log.warning(f'response.status: {response.status}')
                        return
                    data = await response.json()
            except aiohttp.ClientError as e:
                self.log.error(f'Error capturing image: {e!s}')
            except TimeoutError:
                self.log.error('Request timed out')
        if data is None:
            return
        assert 'image' in data
        image_bytes = await rosys.run.cpu_bound(bytes.fromhex, data['image'])
        image = Image(
            camera_id=data['camera_id'],
            size=ImageSize(width=data['width'], height=data['height']),
            time=data['time'],
            data=image_bytes,
            is_broken=data['is_broken'],
            tags=set(data['tags']),
        )
        self._add_image(image)

    async def get_point(self, x, y) -> Point3d | None:
        url = f'http://{self.ip}:{self.port}/point?x={x}&y={y}'
        data: dict[str, Any] | None = None
        async with aiohttp.ClientSession() as session:
            try:
                async with session.get(url, timeout=aiohttp.ClientTimeout(2.0)) as response:
                    if response.status != 200:
                        self.log.warning(f'response.status: {response.status}')
                        return None
                    data = await response.json()
            except aiohttp.ClientError as e:
                self.log.error(f'Error capturing image: {e!s}')
            except TimeoutError:
                self.log.error('Request timed out')
        if data is None:
            return None
        assert 'x' in data
        assert 'y' in data
        assert 'z' in data
        return Point3d(**data)

    @property
    def is_connected(self) -> bool:
        # TODO: check it in capture_image
        return self.connected

    def setup_calibration(self, camera_dict: dict) -> None:
        assert 'resolution' in camera_dict
        assert 'calibration' in camera_dict
        assert 'left_cam' in camera_dict['calibration']
        width = camera_dict['resolution'][0]
        height = camera_dict['resolution'][1]
        fx = camera_dict['calibration']['left_cam']['fx']
        fy = camera_dict['calibration']['left_cam']['fy']
        self.focal_length = (fx + fy) / 2.0
        fy = camera_dict['calibration']['left_cam']['fy']
        cx = camera_dict['calibration']['left_cam']['cx']
        cy = camera_dict['calibration']['left_cam']['cy']
        k1 = camera_dict['calibration']['left_cam']['k1']
        k2 = camera_dict['calibration']['left_cam']['k2']
        p1 = camera_dict['calibration']['left_cam']['p1']
        p2 = camera_dict['calibration']['left_cam']['p2']
        k3 = camera_dict['calibration']['left_cam']['k3']

        size = ImageSize(width=width, height=height)
        K: list[list[float]] = [[fx, 0.0, cx],
                                [0.0, fy, cy],
                                [0.0, 0.0, 1.0]]
        D: list[float] = [k1, k2, p1, p2, k3]
        intrinsics = Intrinsics(matrix=K, distortion=D, size=size)
        self.calibration = Calibration(intrinsics=intrinsics)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        logging.info(f'zedxmini - from_dict - data: {data}')
        return cls(**(data | {
            'calibration': persistence.from_dict(rosys.vision.Calibration, data['calibration']) if data.get('calibration') else None,
        }))

    def to_dict(self) -> dict:
        logging.info('zedxmini - to_dict')
        return super().to_dict() | {
            'focal_length': self.focal_length,
        }
