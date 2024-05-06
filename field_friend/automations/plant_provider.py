import logging
import uuid
from dataclasses import dataclass

import rosys
from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Plant:
    id: str = ...
    type: str
    position: Point
    detection_time: float
    confidence: float = 0.0

    def __post_init__(self) -> None:
        """Generate a unique ID if not already loaded from persistence"""
        if self.id == ...:
            self.id = str(uuid.uuid4())


def check_if_plant_exists(plant: Plant, plants: list[Plant], distance: float) -> bool:
    for p in plants:
        if p.position.distance(plant.position) < distance and p.type == plant.type:
            if p.confidence > plant.confidence:
                return True
            p.position = plant.position
            p.confidence = plant.confidence
            return True
    return False


class PlantProvider:

    def __init__(self) -> None:
        self.log = logging.getLogger('field_friend.plant_provider')
        self.weeds: list[Plant] = []
        self.crops: list[Plant] = []

        self.PLANTS_CHANGED = rosys.event.Event()
        """The collection of plants has changed."""

        self.ADDED_NEW_WEED = rosys.event.Event()
        """A new weed has been added."""

        self.ADDED_NEW_CROP = rosys.event.Event()
        """A new crop has been added."""

        rosys.on_repeat(self.prune, 10.0)

    def prune(self) -> None:
        weeds_max_age = 10.0
        crops_max_age = 60.0 * 60.0
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - weeds_max_age]
        self.crops[:] = [crop for crop in self.crops if crop.detection_time > rosys.time() - crops_max_age]
        self.PLANTS_CHANGED.emit()

    async def add_weed(self, weed: Plant) -> None:
        if await rosys.run.cpu_bound(check_if_plant_exists, weed, self.weeds, 0.05):
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

    async def add_crop(self, crop: Plant) -> None:
        if await rosys.run.cpu_bound(check_if_plant_exists, crop, self.crops, 0.05):
            return
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
