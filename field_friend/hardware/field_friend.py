import numpy as np
import rosys

from .axis import Axis
from .chain_axis import ChainAxis
from .flashlight import Flashlight
from .flashlight_pwm import FlashlightPWM
from .flashlight_v2 import FlashlightV2
from .safety import Safety
from .tornado import Tornado


class FieldFriend(rosys.hardware.Robot):
    MOTOR_GEAR_RATIO = 12.52
    WHEEL_DIAMETER = 0.041 * 17
    M_PER_TICK = WHEEL_DIAMETER / MOTOR_GEAR_RATIO
    WORK_X = 0.118
    WORK_Y = 0.0
    DRILL_RADIUS = 0.025
    CHOP_RADIUS = 0.07
    WORK_X_CHOP = 0.04
    TORNADO_INNER_MAX_DIAMETER = 0.069
    TORNADO_INNER_MIN_DIAMETER = 0.105
    TORNADO_OUTER_MAX_DIAMETER = 0.129
    TORNADO_OUTER_MIN_DIAMETER = 0.165

    def __init__(
            self, *,
            implement_name: str | None,
            wheels: rosys.hardware.Wheels,
            flashlight: Flashlight | FlashlightV2 | FlashlightPWM | None,
            y_axis: Axis | ChainAxis | None,
            z_axis: Axis | Tornado | None,
            imu: rosys.hardware.Imu | None,
            estop: rosys.hardware.EStop,
            bumper: rosys.hardware.Bumper | None,
            bms: rosys.hardware.Bms,
            safety: Safety,
            **kwargs) -> None:
        super().__init__(**kwargs)
        self.implement_name = implement_name
        self.wheels = wheels
        self.flashlight = flashlight
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.imu = imu
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

    def can_reach(self, local_point: rosys.geometry.Point, *, add_work_offset: bool = True, second_tool: bool = False) -> bool:
        """Check if the given point is reachable by the tool.

        The point is given in local coordinates, i.e. the origin is the center of the tool.
        """
        if not self.implement_name:
            raise NotImplementedError('This robot has no tool to reach with.')
        if add_work_offset:
            local_point.x += self.WORK_X
            local_point.y += self.WORK_Y
        if self.implement_name in ['weed_screw', 'tornado'] and isinstance(self.y_axis, Axis):
            return self.y_axis.min_position <= local_point.y <= self.y_axis.max_position
        if self.implement_name in ['dual_mechanism'] and isinstance(self.y_axis, ChainAxis):
            if second_tool:
                return self.y_axis.MIN_POSITION <= local_point.y <= self.y_axis.MAX_POSITION
            return self.y_axis.min_position <= local_point.y <= self.y_axis.max_position
        raise NotImplementedError(f'Tool {self.implement_name} is not implemented for reachability check')

    def tornado_diameters(self, angle: float) -> tuple:
        angle = np.clip(angle, 0, 180)

        def calculate_diameter(max_diameter, min_diameter):
            return (max_diameter - min_diameter) / 2 * (1 - np.cos(np.radians(angle))) + min_diameter

        inner_diameter = calculate_diameter(self.TORNADO_INNER_MAX_DIAMETER, self.TORNADO_INNER_MIN_DIAMETER)
        outer_diameter = calculate_diameter(self.TORNADO_OUTER_MAX_DIAMETER, self.TORNADO_OUTER_MIN_DIAMETER)
        return (inner_diameter, outer_diameter)
