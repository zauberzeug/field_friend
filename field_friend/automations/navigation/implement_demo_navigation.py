# pylint: disable=duplicate-code
# TODO: refactor this and navigation.py
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np
import rosys

from ...hardware import Axis
from ..implements.implement import Implement
from ..implements.weeding_implement import WeedingImplement
from .navigation import Navigation, WorkflowException

if TYPE_CHECKING:
    from ...system import System


class ImplementDemoNavigation(Navigation):

    def __init__(self, system: System, tool: Implement) -> None:
        super().__init__(system, tool)
        self.name = 'Implement Demo'

    @property
    def has_waypoints(self) -> bool:
        return True  # NOTE: this is a demo navigation, we want an infinite automation

    async def prepare(self) -> bool:
        await super().prepare()
        if not isinstance(self.implement, WeedingImplement) and self.system.field_friend.y_axis is not None:
            rosys.notify('Implement Demo only works with a weeding implement', 'negative')
            return False
        await self.implement.activate()
        assert isinstance(self.implement, WeedingImplement)
        self.implement.puncher.is_demo = True
        return True

    async def start(self) -> None:
        try:
            await self.implement.stop_workflow()
            if not await self.implement.prepare():
                self.log.error('Tool-Preparation failed')
                return
            if not await self.prepare():
                self.log.error('Preparation failed')
                return
            self.log.info('Navigation started')
            while self.has_waypoints:
                # TODO: implement has no attribute next_punch_y_position, only weeding implement has it
                # what is the correct way to handle this? currently it's initialized with a recorder as the implement
                assert isinstance(self.implement, WeedingImplement)
                assert isinstance(self.system.field_friend.y_axis, Axis)
                y_min = self.system.field_friend.y_axis.min_position + \
                    (self.system.config.measurements.work_y or 0.0)
                y_max = self.system.field_friend.y_axis.max_position - \
                    (self.system.config.measurements.work_y or 0.0)
                self.implement.next_punch_y_position = np.random.uniform(y_min, y_max)
                self.log.warning(f'next_punch_y_position: {self.implement.next_punch_y_position}')
                await self.implement.start_workflow()
        except WorkflowException as e:
            self.log.error(f'WorkflowException: {e}')
        finally:
            await self.implement.finish()
            await self.finish()
            await self.implement.deactivate()
            await self.driver.wheels.stop()

    def generate_path(self):
        return []

    def backup_to_dict(self) -> dict[str, Any]:
        return super().backup_to_dict() | {
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        return None
