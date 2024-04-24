from typing import Union

import rosys

from .chain_axis import ChainAxisHardware
from .double_wheels import DoubleWheelsHardware
from .flashlight import FlashlightHardware
from .flashlight_pwm import FlashlightPWMHardware
from .flashlight_pwm_v2 import FlashlightPWMHardwareV2
from .flashlight_v2 import FlashlightHardwareV2
from .safety import Safety
from .tornado import TornadoHardware
from .y_axis import YAxisHardware
from .y_axis_canopen import YAxisCanOpenHardware
from .y_axis_tornado import YAxisHardwareTornado
from .z_axis import ZAxisHardware
from .z_axis_canopen import ZAxisCanOpenHardware
from .z_axis_v2 import ZAxisHardwareV2


class SmallSafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: Union[rosys.hardware.WheelsHardware, DoubleWheelsHardware],
                 estop: rosys.hardware.EStopHardware,
                 y_axis: Union[YAxisHardware, ChainAxisHardware,
                               YAxisHardwareTornado, YAxisCanOpenHardware, None] = None,
                 z_axis: Union[ZAxisHardware, ZAxisHardwareV2, TornadoHardware, ZAxisCanOpenHardware, None] = None,
                 flashlight: Union[FlashlightHardware, FlashlightHardwareV2, FlashlightPWMHardware, FlashlightPWMHardwareV2, None],
                 ) -> None:

        lizard_code = f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        for name in estop.pins:
            lizard_code += f'when estop_{name}.level == 0 then {wheels.name}.speed(0, 0); end\n'

        super().__init__(wheels=wheels,
                         estop=estop,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         flashlight=flashlight,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code)
