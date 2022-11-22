import logging
import random

import numpy as np
import rosys
from rosys.geometry import Point, Point3d
from rosys.vision import Calibration, Camera, CameraProvider, Detections, Detector, SimulatedObject

from hardware import Robot

from .coin import Coin, CoinProvider

MINIMUM_COIN_CONFIDENCE = 0.3


class DetectorError(Exception):
    pass


class CoinDetection:

    def __init__(self, detector: Detector, camera_provider: CameraProvider, coin_provider: CoinProvider,
                 robot: Robot) -> None:
        self.detector = detector
        self.camera_provider = camera_provider
        self.rumex_provider = coin_provider
        self.robot = robot
        self.log = logging.getLogger('field_friend.coin_detection')

    async def check_cam(self) -> None:
        self.log.info('Detecting in cam')
        camera = self.camera_provider.cameras['test_front_cam']
        detections = await self.detect(camera)
        await self.update_coins(detections, camera.calibration)

    async def detect(self, camera: Camera) -> Detections:
        image = camera.latest_captured_image
        if image is None:
            raise DetectorError()
        await self.detector.detect(image)
        if not image.detections:
            raise DetectorError()
        return image.detections

    async def update_coins(self, detection: Detections, calibration: Calibration) -> None:
        coin_detections = [d for d in detection.points if d.confidence >= MINIMUM_COIN_CONFIDENCE]
        if not coin_detections:
            return
        coin_positions = calibration.project_array_from_image(np.array([(d.cx, d.cy) for d in coin_detections]))
        coins: list[Coin] = [
            Coin(position=Point(x=p[0], y=p[1]), id=coin_detections[i].uuid, mac='simulated', detection_time=rosys.time())
            for i, p in enumerate(coin_positions)
        ]
        self.log.info(f'found coins: {coins}')
        await self.rumex_provider.add_coin(*coins)

    def place_simulated_objects(self) -> None:
        self.log.info('Placing simulated objects')
        number_of_coins = random.randint(2, 5)
        coins = [
            SimulatedObject(
                category_name='Rumex',
                position=Point3d(x=random.uniform(-0.3, 1),
                                 y=random.uniform(-0.15, 0.15),
                                 z=0),
                size=None,
            )
            for _ in range(0, number_of_coins)
        ]
        self.detector.simulated_objects = coins

    def remove(self, coin: Coin) -> None:
        self.detector.simulated_objects = [o for o in self.detector.simulated_objects if o.uuid != coin.id]
