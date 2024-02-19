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
from .y_axis_tornado import YAxisTornado
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
            flashlight: Union[Flashlight, FlashlightV2, FlashlightPWM, None],
            y_axis: Union[YAxis, ChainAxis, YAxisTornado, None],
            z_axis: Union[ZAxis, ZAxisV2, Tornado, None],
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
