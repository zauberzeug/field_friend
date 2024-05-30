import abc
import logging
from random import randint
from typing import TYPE_CHECKING, Any

import numpy as np
import rosys

from ..implements import Implement
from ..plant import Plant

if TYPE_CHECKING:
    from system import System


class WorkflowException(Exception):
    pass


class Navigation(rosys.persistence.PersistentModule):

    def __init__(self, system: 'System', implement: Implement) -> None:
        super().__init__()
        self.log = logging.getLogger('field_friend.navigation')
        self.driver = system.driver
        self.odometer = system.odometer
        self.kpi_provider = system.kpi_provider
        self.plant_provider = system.plant_provider
        self.puncher = system.puncher
        self.implement = implement
        self.name = 'Unknown'

        self.drive_backwards_to_start: bool = True
        self.drive_to_start: bool = True

    async def start(self) -> None:
        try:
            await self.implement.activate()
            if isinstance(self.driver.wheels, rosys.hardware.WheelsSimulation) and not rosys.is_test:
                self.create_simulation()
            await self.puncher.clear_view()
            await self._start()
        except WorkflowException as e:
            self.kpi_provider.increment_weeding_kpi('automation_stopped')
            self.log.error(f'WorkflowException: {e}')
        finally:
            self.kpi_provider.increment_weeding_kpi('weeding_completed')
            await self.implement.finish()
            await self.driver.wheels.stop()

    @abc.abstractmethod
    async def _start(self) -> None:
        """Executed to start the automation.

        Returns False if automation can not be started."""

    def clear(self) -> None:
        """Resets the state to initial configuration"""

    def backup(self) -> dict:
        return {
            'drive_backwards_to_start': self.drive_backwards_to_start,
            'drive_to_start': self.drive_to_start,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.drive_backwards_to_start = data.get('drive_backwards_to_start', self.drive_backwards_to_start)
        self.drive_to_start = data.get('drive_to_start', self.drive_to_start)

    def create_simulation(self):
        for i in range(0, 30):
            p = Plant(id=str(i), type='beet', detection_time=rosys.time())
            p.positions.append(self.odometer.prediction.point.polar(0.20*i,
                                                                    self.odometer.prediction.yaw)
                               .polar(randint(-2, 2)*0.01, self.odometer.prediction.yaw+np.pi/2))
            p.confidences.append(0.9)
            self.plant_provider.add_crop(p)

            for j in range(1, 7):
                p = Plant(id=str(i), type='weed', detection_time=rosys.time())
                p.positions.append(self.odometer.prediction.point.polar(0.20*i+randint(-5, 5)*0.01,
                                                                        self.odometer.prediction.yaw)
                                   .polar(randint(-15, 15)*0.01, self.odometer.prediction.yaw + np.pi/2))
                p.confidences.append(0.9)
                self.plant_provider.add_weed(p)
