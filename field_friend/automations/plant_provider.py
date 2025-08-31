import logging
from typing import Any

import rosys
from nicegui import ui
from rosys.event import Event
from rosys.geometry import Point3d

from .plant import Plant

# see field_friend/automations/plant_locator.py
MINIMUM_COMBINED_CROP_CONFIDENCE = 0.9
MINIMUM_COMBINED_WEED_CONFIDENCE = 0.9
MATCH_DISTANCE = 0.05
CROP_SPACING = 0.18


def check_if_plant_exists(plant: Plant, plants: list[Plant], distance: float) -> bool:
    for p in plants:
        if p.position.distance(plant.position) < distance and p.type == plant.type:
            p.confidences.append(plant.confidence)
            p.positions.append(plant.position)
            p.detection_image = plant.detection_image
            p.detection_time = plant.detection_time
            return True
    return False


class PlantProvider(rosys.persistence.Persistable):
    def __init__(self) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.plant_provider')
        self.weeds: list[Plant] = []
        self.crops: list[Plant] = []

        self.match_distance: float = MATCH_DISTANCE
        self.crop_spacing: float = CROP_SPACING
        self.minimum_combined_crop_confidence: float = MINIMUM_COMBINED_CROP_CONFIDENCE
        self.minimum_combined_weed_confidence: float = MINIMUM_COMBINED_WEED_CONFIDENCE

        self.PLANTS_CHANGED: Event[[]] = Event()
        """The collection of plants has changed."""

        self.ADDED_NEW_WEED: Event[Plant] = Event()
        """A new weed has been added."""

        self.ADDED_NEW_CROP: Event[Plant] = Event()
        """A new crop has been added."""

        rosys.on_repeat(self.prune, 10.0)

    def prune(self) -> None:
        weeds_max_age = 10.0
        crops_max_age = 60.0 * 300.0
        num_weeds_before = len(self.weeds)
        num_crops_before = len(self.crops)
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - weeds_max_age]
        self.crops[:] = [crop for crop in self.crops if crop.detection_time > rosys.time() - crops_max_age]
        self.log.debug('Pruned %s weeds and %s crops',
                       num_weeds_before - len(self.weeds),
                       num_crops_before - len(self.crops))
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
        self.ADDED_NEW_WEED.emit(weed)

    def remove_weed(self, weed_id: str) -> None:
        num_weeds_before = len(self.weeds)
        self.weeds[:] = [weed for weed in self.weeds if weed.id != weed_id]
        if len(self.weeds) < num_weeds_before:
            self.log.debug('Removed weed %s', weed_id)
            self.PLANTS_CHANGED.emit()

    def clear_weeds(self) -> None:
        self.log.debug('Clearing all %s weeds', len(self.weeds))
        self.weeds.clear()
        self.PLANTS_CHANGED.emit()

    def add_crop(self, crop: Plant) -> None:
        if check_if_plant_exists(crop, self.crops, self.match_distance):
            return
        self.crops.append(crop)
        self.PLANTS_CHANGED.emit()
        self.ADDED_NEW_CROP.emit(crop)

    def remove_crop(self, crop_id: str) -> None:
        num_crops_before = len(self.crops)
        self.crops[:] = [c for c in self.crops if c.id != crop_id]
        if len(self.crops) < num_crops_before:
            self.log.debug('Removed crop %s', crop_id)
            self.PLANTS_CHANGED.emit()

    def clear_crops(self) -> None:
        self.log.debug('Clearing all %s crops', len(self.crops))
        self.crops.clear()
        self.PLANTS_CHANGED.emit()

    def clear(self) -> None:
        self.clear_weeds()
        self.clear_crops()

    def get_relevant_crops(self, point: Point3d, *, max_distance=0.5, min_confidence: float | None = None) -> list[Plant]:
        if min_confidence is None:
            min_confidence = self.minimum_combined_crop_confidence
        return [c for c in self.crops if c.position.distance(point) <= max_distance and c.confidence >= min_confidence]

    def get_relevant_weeds(self, point: Point3d, *, max_distance=0.5, min_confidence: float | None = None) -> list[Plant]:
        if min_confidence is None:
            min_confidence = self.minimum_combined_weed_confidence
        return [w for w in self.weeds if w.position.distance(point) <= max_distance and w.confidence >= min_confidence]

    def backup_to_dict(self) -> dict[str, Any]:
        data = {
            'match_distance': self.match_distance,
            'crop_spacing': self.crop_spacing,
            'minimum_combined_crop_confidence': self.minimum_combined_crop_confidence,
            'minimum_combined_weed_confidence': self.minimum_combined_weed_confidence,
        }
        return data

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self.match_distance = data.get('match_distance', self.match_distance)
        self.crop_spacing = data.get('crop_spacing', self.crop_spacing)
        self.minimum_combined_crop_confidence = data.get('minimum_combined_crop_confidence',
                                                         self.minimum_combined_crop_confidence)
        self.minimum_combined_weed_confidence = data.get('minimum_combined_weed_confidence',
                                                         self.minimum_combined_weed_confidence)

    def settings_ui(self) -> None:
        ui.number('Combined crop confidence threshold', step=0.05, min=0.05, max=5.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_combined_crop_confidence') \
            .tooltip(f'Needed crop confidence for punching (default: {MINIMUM_COMBINED_CROP_CONFIDENCE:.2f})')
        ui.number('Combined weed confidence threshold', step=0.05, min=0.05, max=5.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined') \
            .classes('w-24') \
            .bind_value(self, 'minimum_combined_weed_confidence') \
            .tooltip(f'Needed weed confidence for punching (default: {MINIMUM_COMBINED_WEED_CONFIDENCE:.2f})')
        ui.number('Crop match distance', step=0.01, min=0.01, max=0.10, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'match_distance') \
            .tooltip(f'Maximum distance for a detection to be considered the same plant (default: {MATCH_DISTANCE:.2f})')
        ui.number('Crop spacing', step=0.01, min=0.01, max=1.00, format='%.2f', on_change=self.request_backup) \
            .props('dense outlined suffix=m') \
            .classes('w-24') \
            .bind_value(self, 'crop_spacing') \
            .tooltip(f'Spacing between crops needed for crop position prediction (default: {CROP_SPACING:.2f})')
