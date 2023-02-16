import logging
import random

import rosys
from rosys.geometry import Point, Point3d

from ..hardware import FieldFriend
from .plant import Plant
from .plant_provider import PlantProvider

WEED_CATEGORY_NAME = ['coin', 'weed']
CROP_CATEGORY_NAME = ['coin_with_hole', 'crop']
MINIMUM_WEED_CONFIDENCE = 0.6
MINIMUM_CROP_CONFIDENCE = 0.6


class DetectorError(Exception):
    pass


class PlantDetection:

    def __init__(self, detector: rosys.vision.Detector, plant_provider: PlantProvider, field_friend: FieldFriend) -> None:
        self.detector = detector
        self.plant_provider = plant_provider
        self.field_friend = field_friend
        self.log = logging.getLogger('field_friend.plant_detection')

    async def check_cam(self, camera: rosys.vision.Camera) -> None:
        self.log.info('detecting in cam')
        detections = await self.detect(camera)
        await self.update_plants(detections, camera)

    async def detect(self, camera: rosys.vision.Camera) -> rosys.vision.Detections:
        self.image = camera.latest_captured_image
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

        crop_detections = [
            d for d in detection.points
            if d.category_name in CROP_CATEGORY_NAME and d.confidence >= MINIMUM_CROP_CONFIDENCE]
        if not crop_detections:
            return
        crop_image_positions = [
            rosys.geometry.Point(x=d.cx, y=d.cy) for d in crop_detections
        ]
        crop_world_positions: list[Point3d] = [
            self.calibration.project_from_image(b) for b in crop_image_positions
        ]
        crops: list[Plant] = [
            Plant(position=Point(x=p.x, y=p.y), id=crop_detections[i].uuid, type='crop', mac='detected', detection_time=rosys.time())
            for i, p in enumerate(crop_world_positions)
        ]
        self.log.info(f'found {len(crops)} crops')
        await self.plant_provider.add_crop(*crops)

    def place_simulated_objects(self) -> None:
        self.log.info('Placing simulated objects')
        self.detector.simulated_objects.clear()
        number_of_weeds = random.randint(1, 2)
        number_of_crops = 1  # random.randint(1, 3)
        weeds = [
            rosys.vision.SimulatedObject(
                category_name='weed',
                position=Point3d(x=random.uniform(0.25, 0.35),
                                 y=random.uniform(-0.12, 0.12),
                                 z=0),
                size=None,
            )
            for _ in range(0, number_of_weeds)
        ]
        beets = [
            rosys.vision.SimulatedObject(
                category_name='crop',
                position=Point3d(x=random.uniform(0.25, 0.35),
                                 y=random.uniform(-0.03, 0.03),
                                 z=0),
                size=None,
            )
            for _ in range(0, number_of_crops)
        ]
        self.detector.simulated_objects = weeds + beets

    def remove(self, weed: Plant) -> None:
        self.detector.simulated_objects = [o for o in self.detector.simulated_objects if o.uuid != weed.id]
