import logging
import random

import numpy as np
import rosys
from rosys.geometry import Point, Point3d

from hardware import Robot

from .plant import Plant, PlantProvider

WEED_CATEGORY_NAME = ['coin']
MINIMUM_WEED_CONFIDENCE = 0.3


class DetectorError(Exception):
    pass


class PlantDetection:

    def __init__(self, detector: rosys.vision.Detector, camera_provider: rosys.vision.CameraProvider,
                 plant_provider: PlantProvider, robot: Robot) -> None:
        self.detector = detector
        self.camera_provider = camera_provider
        self.plant_provider = plant_provider
        self.robot = robot
        self.log = logging.getLogger('field_friend.plant_detection')

    async def check_cam(self) -> None:
        self.log.info('Detecting in cam')
        detections = await self.detect()
        await self.update_plants(detections, list(self.camera_provider.cameras.values())[0].calibration)

    async def detect(self) -> rosys.vision.Detections:
        self.image = self.camera_provider.images[0]
        self.log.info(f'getting last captured image')
        if self.image is None:
            self.log.info('no image found')
            raise DetectorError()
        await self.detector.detect(self.image)
        if not self.image.detections:
            self.log.info('no detection found')
            raise DetectorError()
        return self.image.detections

    async def update_plants(self, detection: rosys.vision.Detections, calibration: rosys.vision.Calibration) -> None:
        weed_detections = [
            d for d in detection.points
            if d.category_name in WEED_CATEGORY_NAME and d.confidence >= MINIMUM_WEED_CONFIDENCE]
        if not weed_detections:
            return
        point2d = [
            rosys.geometry.Point(x=d.cx, y=d.cy) for d in weed_detections
        ]
        self.log.info(f'my points in 2d {point2d}')
        weed_positions: list[Point3d] = [
            calibration.project_from_image(point2d[0])  # for p in point2d
        ]
        self.log.info(f'my points after calibration {weed_positions}')
        weeds: list[Plant] = [
            Plant(position=Point(x=p.x, y=p.y), id=weed_detections[i].uuid, type='weed', mac='detected', detection_time=rosys.time())
            for i, p in enumerate(weed_positions)
        ]
        self.log.info(f'found weeds: {weeds}')
        await self.plant_provider.add_weed(*weeds)
        self.log.info(f'weed_provider: {self.plant_provider.weeds}')

    def place_simulated_objects(self) -> None:
        self.log.info('Placing simulated objects')
        number_of_weeds = random.randint(2, 5)
        weeds = [
            rosys.vision.SimulatedObject(
                category_name='weed',
                position=Point3d(x=random.uniform(-0.3, 1),
                                 y=random.uniform(-0.15, 0.15),
                                 z=0),
                size=None,
            )
            for _ in range(0, number_of_weeds)
        ]
        self.detector.simulated_objects = weeds

    def remove(self, weed: Plant) -> None:
        self.detector.simulated_objects = [o for o in self.detector.simulated_objects if o.uuid != weed.id]
