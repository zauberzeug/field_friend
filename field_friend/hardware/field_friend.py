from typing import Union

import numpy as np
import rosys

from .chain_axis import ChainAxis
from .flashlight import Flashlight
from .flashlight_v2 import FlashlightV2
from .safety import Safety
from .y_axis import YAxis
from .z_axis import ZAxis
from .z_axis_v2 import ZAxisV2


class FieldFriend(rosys.hardware.Robot):
    MOTOR_GEAR_RATIO = 12.52
    WHEEL_DIAMETER = 0.041 * 17 / np.pi
    M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO
    WORK_X = 0.118
    DRILL_RADIUS = 0.025
    CHOP_RADIUS = 0.07
    WORK_X_CHOP = 0.04
    WORK_X_DRILL = 0.175

    def __init__(
            self, *,
            version: str,
            wheels: rosys.hardware.Wheels,
            flashlight: Union[Flashlight, FlashlightV2, None],
            y_axis: Union[YAxis, ChainAxis, None],
            z_axis: Union[ZAxis, ZAxisV2, None],
            estop: rosys.hardware.EStop,
            bumper: Union[rosys.hardware.Bumper, None],
            bms: rosys.hardware.Bms,
            safety: Safety,
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.version = version
        self.wheels = wheels
        self.flashlight = flashlight
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.estop = estop
        self.bumper = bumper
        self.bms = bms
        self.safety = safety
        rosys.on_shutdown(self.stop)

    async def stop(self) -> None:
        await self.wheels.stop()
        if self.y_axis:
            await self.y_axis.stop()
        if self.z_axis:
            await self.z_axis.stop()
        if self.flashlight:
            await self.flashlight.turn_off()

    def can_reach(self, local_point: rosys.geometry.Point, second_tool: bool = False) -> bool:
        """Check if the given point is reachable by the tool.

        The point is given in local coordinates, i.e. the origin is the center of the tool.
        """
        if self.version in ['ff3', 'u4']:
            return self.WORK_X - self.DRILL_RADIUS <= local_point.x <= self.WORK_X + self.DRILL_RADIUS \
                and self.y_axis.MIN_POSITION <= local_point.y <= self.y_axis.MAX_POSITION
        elif self.version in ['u2', 'u3']:
            if second_tool == False:
                work_x = self.WORK_X_CHOP
                tool_radius = self.CHOP_RADIUS
                return work_x - tool_radius <= local_point.x <= work_x + tool_radius \
                    and self.y_axis.MIN_POSITION <= local_point.y <= self.y_axis.MAX_POSITION
            else:
                work_x = self.WORK_X_DRILL
                tool_radius = self.DRILL_RADIUS
                return work_x - tool_radius <= local_point.x <= work_x + tool_radius \
                    and self.y_axis.MIN_POSITION+self.y_axis.WORK_OFFSET <= local_point.y <= self.y_axis.MAX_POSITION-self.y_axis.WORK_OFFSET
        else:
            raise NotImplementedError(f'Version {self.version} not implemented')
