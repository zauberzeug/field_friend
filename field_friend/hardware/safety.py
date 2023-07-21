import abc
from typing import Union

import rosys

from .chain_axis import ChainAxis, ChainAxisHardware, ChainAxisSimulation
from .flashlight import Flashlight, FlashlightHardware, FlashlightSimulation
from .flashlight_v2 import FlashlightHardwareV2, FlashlightSimulationV2, FlashlightV2
from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation
from .z_axis_v2 import ZAxisHardwareV2, ZAxisSimulationV2, ZAxisV2


class Safety(rosys.hardware.Module, abc.ABC):
    """The safety module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Union[YAxis, ChainAxis, None] = None,
                 z_axis: Union[ZAxis, ZAxisV2, None] = None,
                 flashlight: Union[Flashlight, FlashlightV2] = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.estop = estop
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.flashlight = flashlight


class SafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Union[YAxisHardware, ChainAxisHardware, None] = None,
                 z_axis: Union[ZAxisHardware, ZAxisHardwareV2, None] = None,
                 flashlight: Union[FlashlightHardware, FlashlightHardwareV2, None]) -> None:

        lizard_code = f'let stop do {wheels.name}.speed(0, 0);'
        if y_axis is not None:
            lizard_code += f' {y_axis.name}.stop();'
        if z_axis is not None:
            lizard_code += f' {z_axis.name}.stop();'
        if isinstance(flashlight, FlashlightHardware):
            lizard_code += f' {flashlight.name}.on();'
        elif isinstance(flashlight, FlashlightHardwareV2):
            lizard_code += f' {flashlight.name}_front.off(); {flashlight.name}_back.off();'
        lizard_code += 'end\n'
        for name in estop.pins:
            lizard_code += f'when estop_{name}.level == 0 then stop(); end\n'
        if isinstance(y_axis, ChainAxisHardware):
            lizard_code += f'when {y_axis.name}_ref_t.level == 1 then {wheels.name}.speed(0, 0); end\n'
        if isinstance(z_axis, ZAxisHardware) or isinstance(z_axis, ZAxisHardwareV2):
            lizard_code += f'when {z_axis.name}_ref_t.level == 1 then {wheels.name}.speed(0, 0); {y_axis.name}.stop(); end\n'

        lizard_code += f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        lizard_code += f'when core.last_message_age > 20000 then stop(); end\n'
        super().__init__(wheels=wheels,
                         estop=estop,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         flashlight=flashlight,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code)


class SafetySimulation(Safety, rosys.hardware.ModuleSimulation):
    """This module implements safety simulation."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Union[YAxisSimulation, ChainAxisSimulation, None] = None,
                 z_axis: Union[ZAxisSimulation, ZAxisSimulationV2, None] = None,
                 flashlight: Union[FlashlightSimulation, FlashlightSimulationV2, None]) -> None:
        super().__init__(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)

    async def step(self, dt: float) -> None:
        if self.estop.active:
            self.wheels.stop()
            if self.y_axis is not None:
                self.y_axis.stop()
            if self.z_axis is not None:
                self.z_axis.stop()
            if self.flashlight is not None:
                self.flashlight.turn_off()
