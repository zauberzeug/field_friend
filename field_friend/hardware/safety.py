import abc

import rosys

from .axis import Axis, AxisSimulation
from .axis_d1 import AxisD1
from .chain_axis import ChainAxis, ChainAxisHardware, ChainAxisSimulation
from .double_wheels import DoubleWheelsHardware
from .flashlight import Flashlight, FlashlightHardware, FlashlightSimulation
from .flashlight_pwm import FlashlightPWM, FlashlightPWMHardware, FlashlightPWMSimulation
from .flashlight_pwm_v2 import FlashlightPWMHardwareV2, FlashlightPWMSimulationV2, FlashlightPWMV2
from .flashlight_v2 import FlashlightHardwareV2, FlashlightSimulationV2, FlashlightV2
from .tornado import Tornado, TornadoHardware, TornadoSimulation
from .y_axis_canopen_hardware import YAxisCanOpenHardware
from .y_axis_stepper_hardware import YAxisStepperHardware
from .z_axis_canopen_hardware import ZAxisCanOpenHardware
from .z_axis_stepper_hardware import ZAxisStepperHardware


class Safety(rosys.hardware.Module, abc.ABC):
    """The safety module is a simple example for a representation of real or simulated robot hardware."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: Axis | ChainAxis | None = None,
                 z_axis: Axis | Tornado | None = None,
                 flashlight: Flashlight | FlashlightV2 | FlashlightPWM | FlashlightPWMV2 | None = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.wheels = wheels
        self.estop = estop
        self.y_axis = y_axis
        self.z_axis = z_axis
        self.flashlight = flashlight


class SafetyHardware(Safety, rosys.hardware.ModuleHardware):
    """This module implements safety hardware."""

    # TODO: add support for AxisD1
    def __init__(self, robot_brain: rosys.hardware.RobotBrain, *,
                 wheels: rosys.hardware.WheelsHardware | DoubleWheelsHardware,
                 estop: rosys.hardware.EStopHardware,
                 bumper: rosys.hardware.BumperHardware | None = None,
                 y_axis: ChainAxisHardware | YAxisStepperHardware | YAxisCanOpenHardware | AxisD1 | None = None,
                 z_axis: ZAxisCanOpenHardware | ZAxisStepperHardware | TornadoHardware | ZAxisCanOpenHardware | AxisD1 | None = None,
                 flashlight: FlashlightHardware | FlashlightHardwareV2 | FlashlightPWMHardware | FlashlightPWMHardwareV2 | None,
                 ) -> None:
        self.estop_active = False
        # implement lizard stop method for available hardware
        lizard_code = f'let stop do {wheels.name}.speed(0, 0);'
        if y_axis is not None:
            if isinstance(y_axis, AxisD1):
                lizard_code += f'{y_axis.name}_motor.stop();'
            else:
                lizard_code += f' {y_axis.name}.stop();'
        if z_axis is not None:
            if isinstance(z_axis, TornadoHardware):
                lizard_code += f'{z_axis.name}_z.stop();'
                lizard_code += f'{z_axis.name}_motor_turn.speed(0);'
            elif isinstance(z_axis, AxisD1):
                lizard_code += f'{z_axis.name}_motor.stop();'
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
            for name in bumper.pins:
                lizard_code += f'when bumper_{name}.level == 1 then stop(); end\n'

        # implement stop call for "ground check" reference sensors
        if isinstance(y_axis, ChainAxisHardware):
            lizard_code += f'when {y_axis.name}_ref_t.level == 1 then {wheels.name}.speed(0, 0); end\n'
        if isinstance(z_axis, TornadoHardware):
            if isinstance(y_axis, YAxisStepperHardware | YAxisCanOpenHardware):
                lizard_code += f'when {z_axis.name}_ref_knife_ground.active == false then \
                    {wheels.name}.speed(0, 0); {y_axis.name}.stop(); end\n'

        # implement watchdog for rosys modules
        lizard_code += f'when core.last_message_age > 1000 then {wheels.name}.speed(0, 0); end\n'
        lizard_code += 'when core.last_message_age > 20000 then stop(); end\n'

        if bumper is not None:
            bumper.BUMPER_TRIGGERED.register(self.bumper_safety_notifications)
        if estop is not None:
            estop.ESTOP_TRIGGERED.register(self.estop_triggered_safety_notifications)
            estop.ESTOP_RELEASED.register(self.estop_released_safety_notifications)

        super().__init__(wheels=wheels,
                         estop=estop,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         flashlight=flashlight,
                         robot_brain=robot_brain,
                         lizard_code=lizard_code)

    def bumper_safety_notifications(self, pin: str) -> None:
        if self.estop_active:
            return
        if pin == 'front_top':
            rosys.notify('Front top bumper triggered', 'warning')
        elif pin == 'front_bottom':
            rosys.notify('Front bottom bumper triggered', 'warning')
        elif pin == 'back':
            rosys.notify('Back bumper triggered', 'warning')

    def estop_triggered_safety_notifications(self) -> None:
        rosys.notify('E-Stop triggered', 'warning')
        self.estop_active = True

    async def estop_released_safety_notifications(self) -> None:
        rosys.notify('E-Stop released')
        await rosys.sleep(0.1)
        self.estop_active = False


class SafetySimulation(Safety, rosys.hardware.ModuleSimulation):
    """This module implements safety simulation."""

    def __init__(self, *,
                 wheels: rosys.hardware.Wheels,
                 estop: rosys.hardware.EStop,
                 y_axis: AxisSimulation | ChainAxisSimulation | None = None,
                 z_axis: AxisSimulation | TornadoSimulation | None = None,
                 flashlight: FlashlightSimulation | FlashlightSimulationV2 | FlashlightPWMSimulation | FlashlightPWMSimulationV2 | None) -> None:
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
