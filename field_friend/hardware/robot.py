import abc
import logging
from typing import Optional

import numpy as np
import rosys
from rosys.event import Event
from rosys.geometry import Velocity
from rosys.hardware import RobotBrain
from rosys.helpers import ramp

import hardware

from .battery import Battery
from .bms import BmsMessage


class Robot(abc.ABC):
    AXIS_HOMING_SPEED: float = 8000
    AXIS_MAX_SPEED: float = 80000
    MIN_Y: float = -0.12
    MAX_Y: float = 0.12
    AXIS_OFFSET_Y = 0.123
    MIN_Z: float = -0.197
    MAX_Z: float = -0.003
    STEPS_PER_MM_YAXIS: float = 666.67
    STEPS_PER_MM_ZAXIS: float = 1600
    AXIS_OFFSET_X = 0.2915

    def __init__(self) -> None:
        self.UPDATED = Event()
        '''the hardware state has been updated'''

        self.VELOCITY_MEASURED = Event()
        '''new velocity measurements are available for processing (argument: list of velocities)'''

        self.ESTOP_TRIGGERED = Event()
        '''estop was triggered'''

        self.log = logging.getLogger('field_friend.robot')

        self.yaxis_end_l: bool = False
        self.yaxis_end_r: bool = False
        self.yaxis_position: int = 0
        self.yaxis_alarm: bool = False
        self.yaxis_idle: bool = False
        self.yaxis_is_referenced: bool = False
        self.yaxis_home_position: int = 0

        self.zaxis_end_t: bool = False
        self.zaxis_end_b: bool = False
        self.zaxis_position: int = 0
        self.zaxis_alarm: bool = False
        self.zaxis_idle: bool = False
        self.zaxis_is_referenced: bool = False
        self.zaxis_home_position: int = 0
        self.zaxis_drill_depth: float = self.MIN_Z

        self.end_stops_active: bool = True
        self.emergency_stop: bool = False
        self.battery: Battery = Battery()

        self.linear_target_speed: float = 0
        self.angular_target_speed:  float = 0

        rosys.on_repeat(self.update, 0.01)
        rosys.on_shutdown(self.stop)

    @property
    def is_real(self) -> bool:
        return isinstance(self, hardware.RobotHardware)

    @property
    def is_simulation(self) -> bool:
        return isinstance(self, hardware.RobotSimulation)

    def depth_to_steps(self, depth: float) -> int:
        steps = int((depth * 1000) * self.STEPS_PER_MM_ZAXIS)
        return steps

    def steps_to_depth(self, steps: int) -> float:
        depth = (steps * (1 / self.STEPS_PER_MM_ZAXIS)) / 1000
        return depth

    def linear_to_steps(self, linear: float) -> int:
        steps = int((linear * 1000) * self.STEPS_PER_MM_YAXIS)
        return steps

    def steps_to_linear(self, steps: int) -> float:
        linear = (steps * (1 / self.STEPS_PER_MM_YAXIS)) / 1000
        return linear

    async def update(self) -> None:
        self.UPDATED.emit()

    async def stop(self) -> None:
        await self.drive(0, 0)
        await self.stop_yaxis()
        await self.stop_zaxis()

    async def start_homing(self) -> bool:
        try:
            if not self.end_stops_active:
                self.log.warning('end stops not activated')
                rosys.notify('end stops not activated')
                return False
            if self.yaxis_end_l or self.yaxis_end_r or self.zaxis_end_b or self.zaxis_end_t:
                self.log.warning('remove from end stops to start homing')
                rosys.notify('remove from end stops to start homing')
                return False
            if not await self.try_reference_zaxis():
                return False
            if not await self.try_reference_yaxis():
                return False
            return True
        finally:
            await self.stop()

    @abc.abstractmethod
    async def drive(self, linear: float, angular: float) -> None:
        self.linear_target_speed = linear
        self.angular_target_speed = angular

    @abc.abstractmethod
    async def stop_yaxis(self) -> None:
        pass

    @abc.abstractmethod
    async def stop_zaxis(self) -> None:
        pass

    @abc.abstractmethod
    async def try_reference_yaxis(self) -> bool:
        return True

    @abc.abstractmethod
    async def try_reference_zaxis(self) -> bool:
        return True

    @abc.abstractmethod
    async def move_yaxis_to(self) -> None:
        pass

    @abc.abstractmethod
    async def move_zaxis_to(self) -> None:
        pass
