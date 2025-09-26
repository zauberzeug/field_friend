from __future__ import annotations

from typing import TYPE_CHECKING

from .weeding_implement import WeedingImplement

if TYPE_CHECKING:
    from ...system import System


class DeltaArm(WeedingImplement):

    def __init__(self, system: System) -> None:
        super().__init__('Delta Arm', system)

    async def start_workflow(self) -> None:
        await super().start_workflow()
