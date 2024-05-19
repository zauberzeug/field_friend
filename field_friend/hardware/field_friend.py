from typing import Union

import numpy as np
import rosys

from .chain_axis import ChainAxis
from .flashlight import Flashlight
from .flashlight_pwm import FlashlightPWM
from .flashlight_v2 import FlashlightV2
from .safety import Safety
from .tornado import Tornado
from .y_axis import YAxis
from .z_axis import ZAxis


class FieldFriend(rosys.hardware.Robot):
    MOTOR_GEAR_RATIO = 12.52
    WHEEL_DIAMETER = 0.041 * 17 / np.pi
    M_PER_TICK = WHEEL_DIAMETER * np.pi / MOTOR_GEAR_RATIO
    WORK_X = 0.118
    DRILL_RADIUS = 0.025
    CHOP_RADIUS = 0.07
    WORK_X_CHOP = 0.04
    TORNADO_INNER_MAX_DIAMETER = 0.069
    TORNADO_INNER_MIN_DIAMETER = 0.105
    TORNADO_OUTER_MAX_DIAMETER = 0.129
    TORNADO_OUTER_MIN_DIAMETER = 0.165

    def __init__(
            self, *,
            tool: str,
            wheels: rosys.hardware.Wheels,
            flashlight: Union[Flashlight, FlashlightV2, FlashlightPWM, None],
            y_axis: Union[YAxis, ChainAxis, None],
            z_axis: Union[ZAxis, Tornado, None],
            estop: rosys.hardware.EStop,
            bumper: Union[rosys.hardware.Bumper, None],
            bms: rosys.hardware.Bms,
            safety: Safety,
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.tool = tool
        self.wheels = wheels
        self.flashlight = flashlight
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.estop = estop
        self.bumper = bumper
        self.bms = bms
        self.safety = safety
        rosys.on_shutdown(self.stop)
        if self.estop:
            self.estop.ESTOP_TRIGGERED.register(self.stop)

    async def stop(self) -> None:
        await self.wheels.stop()
        if self.y_axis:
            await self.y_axis.stop()
        if self.z_axis:
            await self.z_axis.stop()

    def can_reach(self, local_point: rosys.geometry.Point, second_tool: bool = False) -> bool:
        """Check if the given point is reachable by the tool.

        The point is given in local coordinates, i.e. the origin is the center of the tool.
        """
        if self.tool in ['weed_screw', 'tornado'] and isinstance(self.y_axis, YAxis):
            return self.y_axis.min_position <= local_point.y <= self.y_axis.max_position
        elif self.tool in ['dual_mechanism'] and isinstance(self.y_axis, ChainAxis):
            if second_tool:
                return self.y_axis.MIN_POSITION <= local_point.y <= self.y_axis.MAX_POSITION
            else:
                return self.y_axis.min_position <= local_point.y <= self.y_axis.max_position
        else:
            raise NotImplementedError(f'Tool {self.tool} is not implemented for reachability check')

    def tornado_diameters(self, angle: float) -> tuple:
        angle = np.clip(angle, 0, 180)

        def calculate_diameter(max_diameter, min_diameter):
            return (max_diameter - min_diameter) / 2 * (1 - np.cos(np.radians(angle))) + min_diameter

        inner_diameter = calculate_diameter(self.TORNADO_INNER_MAX_DIAMETER, self.TORNADO_INNER_MIN_DIAMETER)
        outer_diameter = calculate_diameter(self.TORNADO_OUTER_MAX_DIAMETER, self.TORNADO_OUTER_MIN_DIAMETER)
        return (inner_diameter, outer_diameter)
