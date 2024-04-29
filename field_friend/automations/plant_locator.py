import asyncio
import logging

import rosys

from .plant_provider import Plant, PlantProvider

WEED_CATEGORY_NAME = ['coin', 'weed', 'big_weed', 'thistle', 'orache', 'weedy_area', ]
CROP_CATEGORY_NAME = ['coin_with_hole', 'crop', 'sugar_beet', 'onion', 'garlic', ]
MINIMUM_WEED_CONFIDENCE = 0.8
MINIMUM_CROP_CONFIDENCE = 0.75


class DetectorError(Exception):
    pass


class PlantLocator:

    def __init__(self,
                 detector: rosys.vision.Detector,
                 plant_provider: PlantProvider,
                 odometer: rosys.driving.Odometer,
                 ) -> None:
        """The PlantLocator manages the whole localization pipeline for plants.

        It pulls images from the configured camera, 
        triggers the plant detection and
        adds the detected plants to the PlantProvider for future use.
        The odometer (eg. current robot position) is used to transform the detected plants from camera coordinates to world coordinates.

        You can change the used camera by setting the camera property.
        If the camera property is None there is no localization being performed.
        """
        self.log = logging.getLogger('field_friend.plant_detection')
        self.camera: rosys.vision.CalibratableCamera | None = None
        self.detector = detector
        self.plant_provider = plant_provider
        self.odometer = odometer
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: list[str] = CROP_CATEGORY_NAME
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE
        rosys.on_repeat(self._detect_plants, 0.01)  # as fast as possible, function will sleep if necessary

    async def _detect_plants(self) -> None:
        if self.camera is None:
            await asyncio.sleep(0.01)
            return
        t = rosys.time()
        if not self.camera:
            raise DetectorError()
        if self.camera.calibration is None:
            raise DetectorError()
        new_image = self.camera.latest_captured_image
        if new_image is None or new_image.detections:
            await asyncio.sleep(0.01)
            return
        await self.detector.detect(new_image)
        if rosys.time() - t < 0.01:  # ensure maximum of 100 Hz
            await asyncio.sleep(0.01 - (rosys.time() - t))
        if not new_image.detections:
            return
            # raise DetetorError()

        # self.log.info(f'{[point.category_name for point in new_image.detections.points]} detections found')
        for d in new_image.detections.points:
            if d.category_name in self.weed_category_names and d.confidence >= self.minimum_weed_confidence:
                # self.log.info('weed found')
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                floor_point = self.camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    self.log.error('could not generate floor point of detection, calibration error')
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                weed = Plant(position=world_point, type=d.category_name, detection_time=rosys.time())
                self.plant_provider.add_weed(weed)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                # self.log.info('crop found')
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                floor_point = self.camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    self.log.error('could not generate floor point of detection, calibration error')
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                crop = Plant(position=world_point, type=d.category_name,
                             detection_time=rosys.time(), confidence=d.confidence)
                self.plant_provider.add_crop(crop)
            elif d.category_name not in self.crop_category_names and d.category_name not in self.weed_category_names:
                self.log.info(f'{d.category_name} not in categories')
            # else:
            #     self.log.info(f'confidence of {d.category_name} to low: {d.confidence}')
