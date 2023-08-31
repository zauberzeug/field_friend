import logging
from typing import Optional

import rosys

from .plant import Plant
from .plant_provider import PlantProvider

WEED_CATEGORY_NAME = ['coin', 'weed']
CROP_CATEGORY_NAME = ['coin_with_hole', 'crop']
MINIMUM_WEED_CONFIDENCE = 0.6
MINIMUM_CROP_CONFIDENCE = 0.6


class DetectorError(Exception):
    pass


class PlantDetector:

    def __init__(self,
                 detector: rosys.vision.Detector,
                 plant_provider: PlantProvider,
                 odometer: rosys.driving.Odometer,
                 ) -> None:
        self.log = logging.getLogger('field_friend.plant_detection')
        self.detector = detector
        self.plant_provider = plant_provider
        self.odometer = odometer
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: list[str] = CROP_CATEGORY_NAME
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE

    async def detect_plants(self, camera: rosys.vision.Camera, *, image: Optional[rosys.vision.Image] = None) -> None:
        self.log.info('detecting plants')

        if camera.calibration is None:
            rosys.notify('camera has no calibration')
            raise DetectorError()
        if image == None:
            # wait for new image
            deadline = rosys.time() + 5.0
            new_image = camera.latest_captured_image
            while new_image == camera.latest_captured_image and rosys.time() < deadline:
                await rosys.sleep(0.1)

            new_image = camera.latest_captured_image
            if new_image is None:
                self.log.info('no image found')
                raise DetectorError()

        await self.detector.detect(new_image)
        if not new_image.detections:
            self.log.info('no detection found')
            raise DetectorError()

        self.log.info(f'{[point.category_name for point in new_image.detections.points]} detections found')
        for d in new_image.detections.points:
            if d.category_name in self.weed_category_names and d.confidence >= self.minimum_weed_confidence:
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                floor_point = camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                weed = Plant(position=world_point, type=d.category_name, detection_time=rosys.time())
                self.plant_provider.add_weed(weed)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                floor_point = camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                crop = Plant(position=world_point, type=d.category_name,
                             detection_time=rosys.time(), confidence=d.confidence)
                self.plant_provider.add_crop(crop)
