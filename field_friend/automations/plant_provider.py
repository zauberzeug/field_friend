import rosys

from .plant import Plant


class PlantProvider:

    def __init__(self) -> None:
        self.weeds: list[Plant] = []
        self.crops: list[Plant] = []

        self.PLANTS_CHANGED = rosys.event.Event()
        """The collection of plants has changed."""

        rosys.on_repeat(self.prune, 10.0)

    def prune(self, max_age: float = 10 * 60.0) -> None:
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - max_age]
        self.crops[:] = [crop for crop in self.crops if crop.detection_time > rosys.time() - max_age]
        self.PLANTS_CHANGED.emit()

    def add_weed(self, weed: Plant) -> None:
        self.weeds.append(weed)
        self.PLANTS_CHANGED.emit()

    def remove_weed(self, weed_id: str) -> None:
        self.weeds[:] = [weed for weed in self.weeds if weed.id != weed_id]
        self.PLANTS_CHANGED.emit()

    def clear_weeds(self) -> None:
        self.weeds.clear()
        self.PLANTS_CHANGED.emit()

    def add_crop(self, crop: Plant) -> None:
        self.crops.append(crop)
        self.PLANTS_CHANGED.emit()

    def remove_crop(self, crop: Plant) -> None:
        self.crops[:] = [c for c in self.crops if c.id != crop.id]
        self.PLANTS_CHANGED.emit()

    def clear_crops(self) -> None:
        self.crops.clear()
        self.PLANTS_CHANGED.emit()

    def clear(self) -> None:
        self.clear_weeds()
        self.clear_crops()
