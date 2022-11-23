import uuid
from dataclasses import dataclass
from typing import Any, Optional

import rosys
from rosys import persistence
from rosys.geometry import Point


@dataclass(slots=True, kw_only=True)
class Plant:
    id: Optional[str] = None
    type: str
    position: Point
    mac: str
    detection_time: float
    outline: Optional[list[tuple[float, float]]] = None

    def __post_init__(self) -> None:
        self.id = self.id or str(uuid.uuid4())

    def __str__(self) -> str:
        return f'plant with id: {self.id[:5]}, pos: {self.position}, mac: {self.mac}'


class PlantProvider:
    def __init__(self) -> None:
        self.weeds: list[Plant] = []
        self.beets: list[Plant] = []

        rosys.on_repeat(self.forget_old_weed, 10.0)

        self.needs_backup: bool = False
        persistence.register(self)

    def backup(self) -> dict:
        return {
            'weed': persistence.to_dict(self.weeds),
            'beets': persistence.to_dict(self.beets)
        }

    def restore(self, data: dict[str, Any]) -> None:
        persistence.replace_list(self.weeds, Plant, data.get('weed', []))
        persistence.replace_list(self.beets, Plant, data.get('beet', []))

    async def forget_old_weed(self) -> None:
        self.weeds[:] = [weed for weed in self.weeds if weed.detection_time > rosys.time() - 3600]
        self.needs_backup = True

    async def add_weed(self, *new: Plant) -> None:
        for weed in new:
            self.weeds.append(weed)
        self.needs_backup = True

    def clear_weeds(self) -> None:
        self.weeds.clear()
        self.needs_backup = True

    def restore_beets(self, data: dict[str, Any]) -> None:
        persistence.replace_list(self.beets, Plant, data.get('beet', []))

    async def forget_old_beet(self) -> None:
        self.beets[:] = [beet for beet in self.beets if beet.detection_time > rosys.time() - 3600]
        self.needs_backup = True

    async def add_beet(self, *new: Plant) -> None:
        for beet in new:
            self.beets.append(beet)
        self.needs_backup = True

    def clear_beets(self) -> None:
        self.beets.clear()
        self.needs_backup = True
