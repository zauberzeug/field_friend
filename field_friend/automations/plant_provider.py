from typing import Any

import rosys
from rosys import persistence

from .plant import Plant


class PlantProvider:
    def __init__(self) -> None:
        self.weeds: list[Plant] = []
        self.crops: list[Plant] = []

        self.PLANTS_CHANGED = rosys.event.Event()
        '''The list of plants has changed'''

        rosys.on_repeat(self.forget_old_weed, 10.0)
        rosys.on_repeat(self.forget_old_crop, 10.0)

        self.needs_backup: bool = False
        # persistence.register(self)

    def backup(self) -> dict:
        return {
            'weed': persistence.to_dict(self.weeds),
            'crop': persistence.to_dict(self.crops)
        }

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_list(self.weeds, Plant, data.get('weed', []))
        persistence.replace_list(self.crops, Plant, data.get('crop', []))
        self.PLANTS_CHANGED.emit()

    def forget_old_weed(self) -> None:
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - 3600]
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()

    def add_weed(self, *new: Plant) -> None:
        for weed in new:
            self.weeds.append(weed)
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()

    def clear_weeds(self) -> None:
        self.weeds.clear()
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()

    def forget_old_crop(self) -> None:
        self.crops[:] = [crop for crop in self.crops if crop.detection_time > rosys.time() - 3600]
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()

    def add_crop(self, *new: Plant) -> None:
        for crop in new:
            self.crops.append(crop)
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()

    def clear_crops(self) -> None:
        self.crops.clear()
        self.needs_backup = True
        self.PLANTS_CHANGED.emit()
