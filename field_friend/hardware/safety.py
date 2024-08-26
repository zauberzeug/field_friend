import abc
from typing import Union

import rosys

from .chain_axis import ChainAxis, ChainAxisHardware, ChainAxisSimulation
from .double_wheels import DoubleWheelsHardware
from .flashlight import Flashlight, FlashlightHardware, FlashlightSimulation
from .flashlight_pwm import (
    FlashlightPWM,
    FlashlightPWMHardware,
    FlashlightPWMSimulation,
)
from .flashlight_pwm_v2 import (
    FlashlightPWMHardwareV2,
    FlashlightPWMSimulationV2,
    FlashlightPWMV2,
)
from .flashlight_v2 import FlashlightHardwareV2, FlashlightSimulationV2, FlashlightV2
from .tornado import Tornado, TornadoHardware, TornadoSimulation
from .y_axis_canopen_hardware import YAxisCanOpenHardware
from .y_axis_stepper_hardware import YAxisStepperHardware
from .axis import Axis, AxisSimulation
from .z_axis_canopen_hardware import ZAxisCanOpenHardware
from .z_axis_stepper_hardware import ZAxisStepperHardware


class Safety(rosys.hardware.Module, abc.ABC):
    """The safety module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Union[Axis, ChainAxis, None] = None,
                 z_axis: Union[Axis, Tornado, None] = None,
                 flashlight: Union[Flashlight, FlashlightV2, FlashlightPWM, FlashlightPWMV2, None] = None,
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
                 wheels: Union[rosys.hardware.WheelsHardware, DoubleWheelsHardware],
                 estop: rosys.hardware.EStopHardware,
                 bumper: Union[rosys.hardware.BumperHardware, None] = None,
                 y_axis: Union[ChainAxisHardware,
                               YAxisStepperHardware, YAxisCanOpenHardware, None] = None,
                 z_axis: Union[ZAxisCanOpenHardware, ZAxisStepperHardware,
                               TornadoHardware, ZAxisCanOpenHardware, None] = None,
                 flashlight: Union[FlashlightHardware, FlashlightHardwareV2, FlashlightPWMHardware, FlashlightPWMHardwareV2, None],
                 ) -> None:

        # implement lizard stop method for available hardware
        lizard_code = f'let stop do {wheels.name}.speed(0, 0);'
        # if y_axis is not None:
        #     lizard_code += f' {y_axis.name}.stop();'
        if z_axis is not None:
            if isinstance(z_axis, TornadoHardware):
                lizard_code += f'{z_axis.name}_z.stop();'
                lizard_code += f'{z_axis.name}_motor_turn.speed(0);'
            else:
                lizard_code += f' {z_axis.name}.stop();'
        if isinstance(flashlight, FlashlightHardware):
            lizard_code += f' {flashlight.name}.on();'
        elif isinstance(flashlight, FlashlightHardwareV2):
            lizard_code += f' {flashlight.name}_front.off(); {flashlight.name}_back.off();'
        lizard_code += 'end\n'

        # implement stop call for estops and bumpers
        for name in estop.pins:
            lizard_code += f'when estop_{name}.level == 0 then stop(); end\n'
        if isinstance(bumper, rosys.hardware.BumperHardware):
            lizard_code += 'when ' + \
                ' or '.join(f'{bumper.name}_{pin}.level == 1' for pin in bumper.pins) + \
                f' then {wheels.name}.off(); end\n'

        # implement stop call for "ground check" reference sensors
        if isinstance(y_axis, ChainAxisHardware):
            lizard_code += f'when {y_axis.name}_ref_t.level == 1 then {wheels.name}.speed(0, 0); end\n'
        if isinstance(z_axis, TornadoHardware):
            if isinstance(y_axis, YAxisStepperHardware) or isinstance(y_axis, YAxisCanOpenHardware):
                lizard_code += f'when {z_axis.name}_ref_knife_ground.active == false then {wheels.name}.speed(0, 0); {y_axis.name}.stop(); end\n'

        # implement watchdog for rosys modules
        lizard_code += f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        lizard_code += 'when core.last_message_age > 20000 then stop(); end\n'

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
                 y_axis: Union[AxisSimulation, ChainAxisSimulation, None] = None,
                 z_axis: Union[AxisSimulation, TornadoSimulation, None] = None,
                 flashlight: Union[FlashlightSimulation, FlashlightSimulationV2, FlashlightPWMSimulation, FlashlightPWMSimulationV2, None]) -> None:
        super().__init__(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)

    async def step(self, dt: float) -> None:
        if self.estop.active:
            await self.wheels.stop()
            if self.y_axis is not None:
                await self.y_axis.stop()
            if self.z_axis is not None:
                await self.z_axis.stop()
            if self.flashlight is not None:
                await self.flashlight.turn_off()
