import abc
from typing import Optional

import rosys

from .y_axis import YAxis, YAxisHardware, YAxisSimulation
from .z_axis import ZAxis, ZAxisHardware, ZAxisSimulation


class Safety(rosys.hardware.Module, abc.ABC):
    """The safety module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Optional[YAxis] = None,
                 z_axis: Optional[ZAxis] = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.estop = estop
        self.y_axis = y_axis
        self.z_axis = z_axis


class SafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Optional[YAxisHardware] = None,
                 z_axis: Optional[ZAxisHardware] = None) -> None:
        if y_axis is not None and z_axis is not None:
            lizard_code = f'let stop do {wheels.name}.speed(0, 0); {y_axis.name}.stop(); {z_axis.name}.stop(); end\n'
        else:
            lizard_code = f'let stop do {wheels.name}.speed(0, 0); end\n'
        for name in estop.pins:
            lizard_code += f'when estop_{name}.level == 0 then stop(); end\n'
        if y_axis is not None and z_axis is not None:
            lizard_code += f'when {z_axis.name}_ref_t.level == 0 then {wheels.name}.speed(0, 0); end\n'
            lizard_code += f'when {z_axis.name}_ref_t.level == 0 then {y_axis.name}.stop(); end\n'
        lizard_code += f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        lizard_code += f'when core.last_message_age > 15000 then stop(); end\n'
        super().__init__(wheels=wheels,
                         estop=estop,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code)


class SafetySimulation(Safety, rosys.hardware.ModuleSimulation):
    """This module implements safety simulation."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Optional[YAxisSimulation] = None,
                 z_axis: Optional[ZAxisSimulation] = None) -> None:
        super().__init__(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis)

    async def step(self, dt: float) -> None:
        if self.estop.active:
            self.wheels.stop()
            if self.y_axis is not None:
                self.y_axis.stop()
            if self.z_axis is not None:
                self.z_axis.stop()
