import asyncio
import logging
from typing import Any

import rosys
from nicegui import ui
from rosys.vision import Autoupload

from ..vision import SimulatedCam
from .plant import Plant
from .plant_provider import PlantProvider

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
            'minimum_weed_confidence': self.minimum_weed_confidence,
            'minimum_crop_confidence': self.minimum_crop_confidence,
            'autoupload': self.autoupload.value,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.minimum_weed_confidence = data.get('minimum_weed_confidence', self.minimum_weed_confidence)
        self.minimum_crop_confidence = data.get('minimum_crop_confidence', self.minimum_crop_confidence)
        self.autoupload = Autoupload(data.get('autoupload', self.autoupload)) \
            if 'autoupload' in data else Autoupload.DISABLED
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
        assert isinstance(camera, rosys.vision.CalibratableCamera)
        if camera.calibration is None:
            self.log.error('no calibration found')
            raise DetectorError()
        new_image = camera.latest_captured_image
        if new_image is None or new_image.detections:
            await asyncio.sleep(0.01)
            return
        await self.detector.detect(new_image, autoupload=self.autoupload, tags=['kiebitz'])
        if rosys.time() - t < 0.01:  # ensure maximum of 100 Hz
            await asyncio.sleep(0.01 - (rosys.time() - t))
        if not new_image.detections:
            return
            # raise DetetorError()

        # self.log.info(f'{[point.category_name for point in new_image.detections.points]} detections found')
        for d in new_image.detections.points:
            image_point = rosys.geometry.Point(x=d.cx, y=d.cy)
            if isinstance(camera, SimulatedCam):
                world_point = camera.calibration.project_from_image(image_point).projection()
            else:
                floor_point = camera.calibration.project_from_image(image_point)
                if floor_point is None:
                    self.log.error('could not generate floor point of detection, calibration error')
                    continue
                world_point = self.odometer.prediction.transform(floor_point.projection())
            if world_point is None:
                self.log.error('could not generate world point of detection, calibration error')
                continue
            plant = Plant(type=d.category_name,
                          detection_time=rosys.time(),
                          detection_image=new_image)
            plant.positions.append(world_point)
            plant.confidences.append(d.confidence)
            if d.category_name in self.weed_category_names and d.confidence >= self.minimum_weed_confidence:
                # self.log.info('weed found')
                await self.plant_provider.add_weed(plant)
            elif d.category_name in self.crop_category_names and d.confidence >= self.minimum_crop_confidence:
                # self.log.info('crop found')
                self.plant_provider.add_crop(plant)
            elif d.category_name not in self.crop_category_names and d.category_name not in self.weed_category_names:
                self.log.info(f'{d.category_name} not in categories')
            # else:
            #     self.log.info(f'confidence of {d.category_name} to low: {d.confidence}')

    def pause(self) -> None:
        if self.is_paused:
            return
        self.log.info('pausing plant detection')
        self.is_paused = True

    def resume(self) -> None:
        if not self.is_paused:
            return
        self.log.info('resuming plant detection')
        self.is_paused = False

    def settings_ui(self) -> None:
        ui.number('Min. weed confidence', format='%.2f', value=0.8, step=0.05, min=0.0, max=1.0) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_weed_confidence') \
            .tooltip('Set the minimum weed confidence for the weeding automation')
        ui.number('Min. crop confidence', format='%.2f', value=0.4, step=0.05, min=0.0, max=1.0) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_crop_confidence') \
            .tooltip('Set the minimum crop confidence for the weeding automation')
        options = [autoupload for autoupload in rosys.vision.Autoupload]
        ui.select(options, label='Autoupload', on_change=self.backup) \
            .bind_value(self, 'autoupload') \
            .classes('w-24').tooltip('Set the autoupload for the weeding automation')
