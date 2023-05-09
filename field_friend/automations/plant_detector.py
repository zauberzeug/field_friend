import logging
import random

import rosys
from rosys.geometry import Point3d

from .plant import Plant
from .plant_provider import PlantProvider

WEED_CATEGORY_NAME = ['coin', 'weed']
CROP_CATEGORY_NAME = ['coin_with_hole', 'crop']
MINIMUM_WEED_CONFIDENCE = 0.6
MINIMUM_CROP_CONFIDENCE = 0.6


class DetectorError(Exception):
    pass


class PlantDetector:

    def __init__(self, detector: rosys.vision.Detector, plant_provider: PlantProvider) -> None:
        self.log = logging.getLogger('field_friend.plant_detection')
        self.detector = detector
        self.plant_provider = plant_provider
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: list[str] = CROP_CATEGORY_NAME
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE

    async def detect_plants(self, camera: rosys.vision.Camera) -> None:
        self.log.info('detecting plants')

        if camera.calibration is None:
            rosys.notify('camera has no calibration')
            raise DetectorError()
        
        image = camera.latest_captured_image
        if image is None:
            self.log.info('no image found')
            raise DetectorError()
        
        await self.detector.detect(image)
        if not image.detections:
            self.log.info('no detection found')
            raise DetectorError()

        for d in image.detections.points:
            if d.category_name in self.weed_category_names and d.confidence >= self.minimum_weed_confidence:
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                world_point = camera.calibration.project_from_image(image_point)
                weed = Plant(position=world_point.projection(), type=d.category_name, detection_time=rosys.time())
                self.plant_provider.add_weed(weed)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                world_point = camera.calibration.project_from_image(image_point)
                crop = Plant(position=world_point.projection(), type=d.category_name, detection_time=rosys.time())
                self.plant_provider.add_crop(crop)

    def add_simulated_objects(self, number_of_weeds: int = 2, number_of_crops: int = 1) -> None:
        self.log.info('Adding simulated objects')
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        for _ in range(number_of_weeds):
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(
                category_name='weed',
                position=Point3d(x=random.uniform(0.25, 0.35), y=random.uniform(-0.12, 0.12), z=0),
            ))
        for _ in range(number_of_crops):
            self.detector.simulated_objects.append(rosys.vision.SimulatedObject(
                category_name='crop',
                position=Point3d(x=random.uniform(0.27, 0.35), y=random.uniform(-0.03, 0.03), z=0),
            ))

    def remove_simulated_object(self, weed: Plant) -> None:
        assert isinstance(self.detector, rosys.vision.DetectorSimulation)
        self.detector.simulated_objects = [o for o in self.detector.simulated_objects if o.uuid != weed.id]
