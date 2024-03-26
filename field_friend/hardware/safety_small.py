import abc
from typing import Union

import rosys

from .chain_axis import ChainAxis, ChainAxisHardware, ChainAxisSimulation
from .flashlight import Flashlight, FlashlightHardware, FlashlightSimulation
from .flashlight_pwm import FlashlightPWM, FlashlightPWMHardware, FlashlightPWMSimulation
from .flashlight_pwm_v2 import FlashlightPWMHardwareV2, FlashlightPWMSimulationV2, FlashlightPWMV2
from .flashlight_v2 import FlashlightHardwareV2, FlashlightSimulationV2, FlashlightV2
from .safety import Safety
from .tornado import Tornado, TornadoHardware, TornadoSimulation
from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .y_axis_tornado import YAxisHardwareTornado, YAxisSimulationTornado, YAxisTornado
from .y_axis_tornado_v2_canopen import YAxisHardwareTornadoV2, YAxisSimulationTornadoV2, YAxisTornadoV2
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation
from .z_axis_v2 import ZAxisHardwareV2, ZAxisSimulationV2, ZAxisV2


class SmallSafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: rosys.hardware.WheelsHardware,
                 estop: rosys.hardware.EStop,
                 bumper: Union[rosys.hardware.BumperHardware, None] = None,
                 y_axis: Union[YAxisHardware, ChainAxisHardware,
                               YAxisHardwareTornado, YAxisHardwareTornadoV2, None] = None,
                 z_axis: Union[ZAxisHardware, ZAxisHardwareV2, TornadoHardware, None] = None,
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
