import logging
from typing import Any

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Point, Pose

from .plant import Plant


def check_if_plant_exists(plant: Plant, plants: list[Plant], distance: float) -> bool:
    for p in plants:
        if p.position.distance(plant.position) < distance and p.type == plant.type:
            p.confidences.append(plant.confidence)
            p.positions.append(plant.position)
            p.detection_image = plant.detection_image
            p.detection_time = plant.detection_time
            return True
    return False


class PlantProvider(rosys.persistence.PersistentModule):
    def __init__(self, persistence_key: str = 'plant_provider') -> None:
        super().__init__(persistence_key=f'field_friend.automations.{persistence_key}')
        self.log = logging.getLogger('field_friend.plant_provider')
        self.weeds: list[Plant] = []
        self.crops: list[Plant] = []

        self.match_distance: float = 0.07
        self.crop_spacing: float = 0.18
        self.predict_crop_position: bool = True
        self.prediction_confidence: float = 0.3
        self.crop_confidence_threshold: float = 0.8
        self.weed_confidence_threshold: float = 0.8

        self.PLANTS_CHANGED = rosys.event.Event()
        """The collection of plants has changed."""

        self.ADDED_NEW_WEED = rosys.event.Event()
        """A new weed has been added."""

        self.ADDED_NEW_CROP = rosys.event.Event()
        """A new crop has been added."""

        rosys.on_repeat(self.prune, 10.0)

    def backup(self) -> dict:
        data = {
            'match_distance': self.match_distance,
            'crop_spacing': self.crop_spacing,
            'predict_crop_position': self.predict_crop_position,
            'prediction_confidence': self.prediction_confidence,
            'crop_confidence_threshold': self.crop_confidence_threshold,
            'weed_confidence_threshold': self.weed_confidence_threshold,
        }
        return data

    def restore(self, data: dict[str, Any]) -> None:
        self.match_distance = data.get('match_distance', self.match_distance)
        self.crop_spacing = data.get('crop_spacing', self.crop_spacing)
        self.predict_crop_position = data.get('predict_crop_position', self.predict_crop_position)
        self.prediction_confidence = data.get('prediction_confidence', self.prediction_confidence)
        self.crop_confidence_threshold = data.get('crop_confidence_threshold', self.crop_confidence_threshold)
        self.weed_confidence_threshold = data.get('weed_confidence_threshold', self.weed_confidence_threshold)

    def prune(self) -> None:
        weeds_max_age = 10.0
        crops_max_age = 60.0 * 300.0
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - weeds_max_age]
        self.crops[:] = [crop for crop in self.crops if crop.detection_time > rosys.time() - crops_max_age]
        self.PLANTS_CHANGED.emit()

    def get_plant_by_id(self, plant_id: str) -> Plant:
        for plant in self.crops + self.weeds:
            if plant.id == plant_id:
                return plant
        raise ValueError(f'Plant with ID {plant_id} not found')

    async def add_weed(self, weed: Plant) -> None:
        if check_if_plant_exists(weed, self.weeds, 0.02):
            return
        self.weeds.append(weed)
        self.PLANTS_CHANGED.emit()
        self.ADDED_NEW_WEED.emit()

    def remove_weed(self, weed_id: str) -> None:
        self.weeds[:] = [weed for weed in self.weeds if weed.id != weed_id]
        self.PLANTS_CHANGED.emit()

    def clear_weeds(self) -> None:
        self.weeds.clear()
        self.PLANTS_CHANGED.emit()

    def add_crop(self, crop: Plant) -> None:
        if check_if_plant_exists(crop, self.crops, self.match_distance):
            return
        if self.predict_crop_position:
            self._add_crop_prediction(crop)
        self.crops.append(crop)
        self.PLANTS_CHANGED.emit()
        self.ADDED_NEW_CROP.emit()

    def remove_crop(self, crop: Plant) -> None:
        self.crops[:] = [c for c in self.crops if c.id != crop.id]
        self.PLANTS_CHANGED.emit()

    def clear_crops(self) -> None:
        self.crops.clear()
        self.PLANTS_CHANGED.emit()

    def clear(self) -> None:
        self.clear_weeds()
        self.clear_crops()

    def _add_crop_prediction(self, plant: Plant) -> None:
        sorted_crops = sorted(self.crops, key=lambda crop: crop.position.distance(plant.position))
        if len(sorted_crops) < 2:
            return
        crop_1 = sorted_crops[0]
        crop_2 = sorted_crops[1]

        yaw = crop_2.position.direction(crop_1.position)
        prediction = crop_1.position.polar(self.crop_spacing, yaw)

        if plant.position.distance(prediction) > self.match_distance:
            return
        plant.positions.append(prediction)
        plant.confidences.append(self.prediction_confidence)

    def get_relevant_crops(self, point: Point, *, max_distance=0.5) -> list[Plant]:
        return [c for c in self.crops if c.position.distance(point) <= max_distance and c.confidence > self.crop_confidence_threshold]

    def get_relevant_weeds(self, point: Point, *, max_distance=0.5) -> list[Plant]:
        return [w for w in self.weeds if w.position.distance(point) <= max_distance and w.confidence > self.weed_confidence_threshold]

    def settings_ui(self) -> None:
        ui.number('Combined crop confidence threshold', step=0.05, min=0.05, max=5.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'crop_confidence_threshold') \
            .tooltip('Needed crop confidence for punshing')
        ui.number('Combined weed confidence threshold', step=0.05, min=0.05, max=5.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'weed_confidence_threshold') \
            .tooltip('Needed weed confidence for punshing')
        ui.number('Crop match distance', step=0.01, min=0.01, max=0.10, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'match_distance') \
            .tooltip('Maximum distance for a detection to be considered the same plant')
        ui.number('Crop spacing', step=0.01, min=0.01, max=1.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'crop_spacing') \
            .tooltip('Spacing between crops needed for crop position prediction')
        ui.number('Crop prediction confidence', step=0.05, min=0.05, max=1.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'prediction_confidence') \
            .tooltip('Confidence of the crop prediction')
        ui.checkbox('Crop Prediction', on_change=self.request_backup) \
            .bind_value(self, 'predict_crop_position') \
            .tooltip('Provides a confidence boost for crop detections that match the expected crop spacing')
