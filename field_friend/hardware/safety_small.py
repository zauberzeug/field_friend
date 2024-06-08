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
from .y_axis_canopen_hardware import YAxisCanOpenHardware
from .y_axis_stepper_hardware import YAxisStepperHardware
from .z_axis_canopen_hardware import ZAxisCanOpenHardware
from .z_axis_stepper_hardware import ZAxisStepperHardware


class SmallSafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: Union[rosys.hardware.WheelsHardware, DoubleWheelsHardware],
                 estop: rosys.hardware.EStopHardware,
                 bumper: Union[rosys.hardware.BumperHardware, None] = None,
                 y_axis: Union[ChainAxisHardware,
                               YAxisStepperHardware, YAxisCanOpenHardware, None] = None,
                 z_axis: Union[TornadoHardware, ZAxisCanOpenHardware, ZAxisStepperHardware, None] = None,
                 flashlight: Union[FlashlightHardware, FlashlightHardwareV2, FlashlightPWMHardware, FlashlightPWMHardwareV2, None],
                 ) -> None:

        lizard_code = f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        for name in estop.pins:
            lizard_code += f'when estop_{name}.level == 0 then {wheels.name}.speed(0, 0); end\n'
        if isinstance(bumper, rosys.hardware.BumperHardware):
            lizard_code += 'when ' + \
                ' or '.join(f'{bumper.name}_{pin}.level == 1' for pin in bumper.pins) + \
                f' then {wheels.name}.off(); end\n'

        super().__init__(wheels=wheels,
                         estop=estop,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         flashlight=flashlight,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code)
