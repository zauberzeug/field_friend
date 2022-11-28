import logging
import random

import rosys
from rosys.geometry import Point, Point3d

from hardware import Robot

from .plant import Plant, PlantProvider

WEED_CATEGORY_NAME = ['coin']
BEET_CATEGORY_NAME = ['coin_with_hole']
MINIMUM_WEED_CONFIDENCE = 0.3
MINIMUM_BEET_CONFIDENCE = 0.3


class DetectorError(Exception):
    pass


class PlantDetection:

    def __init__(self, detector: rosys.vision.Detector, plant_provider: PlantProvider, robot: Robot) -> None:
        self.detector = detector
        self.plant_provider = plant_provider
        self.robot = robot
        self.log = logging.getLogger('field_friend.plant_detection')

    async def check_cam(self, camera: rosys.vision.Camera) -> None:
        self.log.info('Detecting in cam')
        detections = await self.detect(camera)
        await self.update_plants(detections, camera)

    async def detect(self, camera: rosys.vision.Camera) -> rosys.vision.Detections:
        self.image = camera.latest_captured_image
        self.log.info(f'getting last captured image {self.image.time}')
        if self.image is None:
            self.log.info('no image found')
            raise DetectorError()
        await self.detector.detect(self.image)
        if not self.image.detections:
            self.log.info('no detection found')
            raise DetectorError()
        return self.image.detections

    async def update_plants(self, detection: rosys.vision.Detections, camera: rosys.vision.Camera) -> None:
        self.calibration = camera.calibration
        if self.calibration is None:
            rosys.notify('camera has no calibration')
            raise DetectorError()
        weed_detections = [
            d for d in detection.points
            if d.category_name in WEED_CATEGORY_NAME and d.confidence >= MINIMUM_WEED_CONFIDENCE]
        if weed_detections:
            weed_image_positions = [
                rosys.geometry.Point(x=d.cx, y=d.cy) for d in weed_detections
            ]
            weed_world_positions: list[Point3d] = [
                self.calibration.project_from_image(w) for w in weed_image_positions
            ]
            weeds: list[Plant] = [
                Plant(position=Point(x=p.x, y=p.y), id=weed_detections[i].uuid, type='weed', mac='detected', detection_time=rosys.time())
                for i, p in enumerate(weed_world_positions)
            ]
            self.log.info(f'found {len(weeds)} weeds')
            await self.plant_provider.add_weed(*weeds)

        beet_detections = [
            d for d in detection.points
            if d.category_name in BEET_CATEGORY_NAME and d.confidence >= MINIMUM_BEET_CONFIDENCE]
        if not beet_detections:
            return
        beet_image_positions = [
            rosys.geometry.Point(x=d.cx, y=d.cy) for d in beet_detections
        ]
        beet_world_positions: list[Point3d] = [
            self.calibration.project_from_image(b) for b in beet_image_positions
        ]
        beets: list[Plant] = [
            Plant(position=Point(x=p.x, y=p.y), id=beet_detections[i].uuid, type='beet', mac='detected', detection_time=rosys.time())
            for i, p in enumerate(beet_world_positions)
        ]
        self.log.info(f'found {len(beets)} beets')
        await self.plant_provider.add_beet(*beets)

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
