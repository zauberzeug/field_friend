# pylint: disable=duplicate-code
# TODO: refactor this and navigation.py
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys

from ...automations.implements.implement import Implement
from .navigation import Navigation

if TYPE_CHECKING:
    from ...system import System


class WorkflowException(Exception):
    pass


class CrossglideDemoNavigation(Navigation):

    def __init__(self, system: 'System', tool: Implement) -> None:
        super().__init__(system, tool)
        self.MAX_STRETCH_DISTANCE: float = 5.0
        self.detector = system.detector
        self.name = 'Crossglide Demo'
        self.origin: rosys.geometry.Point
        self.target: rosys.geometry.Point

    async def prepare(self) -> bool:
        await super().prepare()
        self.log.info(f'Activating {self.implement.name}...')
        await self.implement.activate()
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
            if isinstance(self.driver.wheels, rosys.hardware.WheelsSimulation) and not rosys.is_test:
                self.create_simulation()
            self.log.info('Navigation started')
            while not self._should_finish():
                # TODO: implement has no attribute next_punch_y_position, only weeding implement has it
                # what is the correct way to handle this? currently it's initialized with a recorder as the implement
                self.implement.next_punch_y_position = np.random.uniform(-0.11, 0.1)
                await self.implement.start_workflow()
        except WorkflowException as e:
            self.log.error(f'WorkflowException: {e}')
        finally:
            await self.implement.finish()
            await self.finish()
            await self.driver.wheels.stop()

    async def finish(self) -> None:
        await super().finish()
        await self.implement.deactivate()

    async def _drive(self, distance: float) -> None:
        pass

    def _should_finish(self) -> bool:
        return False

    def create_simulation(self):
        pass
        # TODO: implement create_simulation

    def backup(self) -> dict:
        return super().backup() | {
        }

    def restore(self, data: dict[str, Any]) -> None:
        return None
