import asyncio
import logging
from typing import Any

import rosys
from rosys.vision import Autoupload

from .plant_provider import Plant, PlantProvider

WEED_CATEGORY_NAME = ['coin', 'weed', 'big_weed', 'thistle', 'orache', 'weedy_area', ]
CROP_CATEGORY_NAME = ['coin_with_hole', 'crop', 'sugar_beet', 'onion', 'garlic', ]
MINIMUM_WEED_CONFIDENCE = 0.8
MINIMUM_CROP_CONFIDENCE = 0.75


class DetectorError(Exception):
    pass


class PlantLocator(rosys.persistence.PersistentModule):

    def __init__(self,
                 camera_provider: rosys.vision.CameraProvider,
                 detector: rosys.vision.Detector,
                 plant_provider: PlantProvider,
                 odometer: rosys.driving.Odometer,
                 ) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.plant_detection')
        self.camera_provider = camera_provider
        self.detector = detector
        self.plant_provider = plant_provider
        self.odometer = odometer
        self.is_paused = True
        self.autoupload: Autoupload = Autoupload.DISABLED
        self.weed_category_names: list[str] = WEED_CATEGORY_NAME
        self.crop_category_names: list[str] = CROP_CATEGORY_NAME
        self.minimum_weed_confidence: float = MINIMUM_WEED_CONFIDENCE
        self.minimum_crop_confidence: float = MINIMUM_CROP_CONFIDENCE
        rosys.on_repeat(self._detect_plants, 0.01)  # as fast as possible, function will sleep if necessary

    def backup(self) -> dict:
        self.log.info(f'backup: autoupload: {self.autoupload}')
        return {
            'weed_category_names': self.weed_category_names,
            'crop_category_names': self.crop_category_names,
            'minimum_weed_confidence': self.minimum_weed_confidence,
            'minimum_crop_confidence': self.minimum_crop_confidence,
            'autoupload': self.autoupload.value,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.weed_category_names = data.get('weed_category_names', self.weed_category_names)
        self.crop_category_names = data.get('crop_category_names', self.crop_category_names)
        self.minimum_weed_confidence = data.get('minimum_weed_confidence', self.minimum_weed_confidence)
        self.minimum_crop_confidence = data.get('minimum_crop_confidence', self.minimum_crop_confidence)
        self.autoupload = Autoupload(data.get('autoupload', self.autoupload)
                                     ) if 'autoupload' in data else Autoupload.DISABLED
        self.log.info(f'self.autoupload: {self.autoupload}')

    async def _detect_plants(self) -> None:
        if self.is_paused:
            await asyncio.sleep(0.01)
            return
        t = rosys.time()
        camera = next((camera for camera in self.camera_provider.cameras.values() if camera.is_connected), None)
        if not camera:
            self.log.error('no connected camera found')
            return
        if camera.calibration is None:
            self.log.error('no calibration found')
            raise DetectorError()
        new_image = camera.latest_captured_image
        if new_image is None or new_image.detections:
            await asyncio.sleep(0.01)
            return
        await self.detector.detect(new_image, autoupload=self.autoupload)
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
                floor_point = camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    self.log.error('could not generate floor point of detection, calibration error')
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                weed = Plant(position=world_point, type=d.category_name, confidence=d.confidence,
                             detection_time=rosys.time(), detection_image=new_image)
                await self.plant_provider.add_weed(weed)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                # self.log.info('crop found')
                image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
                floor_point = camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    self.log.error('could not generate floor point of detection, calibration error')
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
                crop = Plant(position=world_point, type=d.category_name,
                             detection_time=rosys.time(), confidence=d.confidence, detection_image=new_image)
                await self.plant_provider.add_crop(crop)
            elif d.category_name not in self.crop_category_names and d.category_name not in self.weed_category_names:
                self.log.info(f'{d.category_name} not in categories')
            # else:
            #     self.log.info(f'confidence of {d.category_name} to low: {d.confidence}')

    def pause(self) -> None:
        self.log.info('pausing plant detection')
        self.is_paused = True

    def resume(self) -> None:
        self.log.info('resuming plant detection')
        self.is_paused = False
