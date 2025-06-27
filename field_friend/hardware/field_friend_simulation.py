
import logging

import numpy as np
import rosys

# change the config to the config of simulated Robot
from ..config import (
    AxisD1Configuration,
    FieldFriendConfiguration,
    SprayerConfiguration,
    TornadoConfiguration,
    ZCanOpenConfiguration,
    ZStepperConfiguration,
)
from .axis import AxisSimulation
from .chain_axis import ChainAxisSimulation
from .double_wheels import WheelsSimulationWithAcceleration
from .field_friend import FieldFriend
from .flashlight import FlashlightSimulation
from .flashlight_pwm_v2 import FlashlightPWMSimulationV2
from .flashlight_v2 import FlashlightSimulationV2
from .safety import SafetySimulation
from .sprayer import SprayerSimulation
from .tornado import TornadoSimulation


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self, config: FieldFriendConfiguration, *, use_acceleration: bool = False) -> None:
        self.MOTOR_GEAR_RATIO = config.measurements.motor_gear_ratio
        self.TOOTH_COUNT = config.measurements.tooth_count
        self.PITCH = config.measurements.pitch
        self.WHEEL_DIAMETER = self.TOOTH_COUNT * self.PITCH / np.pi
        self.M_PER_TICK = self.WHEEL_DIAMETER * np.pi / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE = config.measurements.wheel_distance
        tool = config.tool
        if tool in ['tornado', 'weed_screw', 'sprayer', None]:
            self.WORK_X = config.measurements.work_x
            if config.measurements.work_y:
                self.WORK_Y = config.measurements.work_y
            self.DRILL_RADIUS = config.measurements.drill_radius
        elif tool in ['dual_mechanism']:
            assert config.measurements.work_x_chop is not None
            self.WORK_X_CHOP = config.measurements.work_x_chop
            self.WORK_X = config.measurements.work_x
            self.DRILL_RADIUS = config.measurements.drill_radius
            assert config.measurements.chop_radius is not None
            self.CHOP_RADIUS = config.measurements.chop_radius
        else:
            logging.warning('Unknown FieldFriend tool: %s', tool)
        wheels = WheelsSimulationWithAcceleration(self.WHEEL_DISTANCE) if use_acceleration else rosys.hardware.WheelsSimulation(self.WHEEL_DISTANCE)

        y_axis: AxisSimulation | ChainAxisSimulation | None
        if not config.y_axis:
            y_axis = None
        elif config.y_axis.version == 'chain_axis':
            y_axis = ChainAxisSimulation()
        elif config.y_axis.version in ['y_axis_stepper', 'y_axis_canopen', 'axis_d1']:
            y_axis = AxisSimulation(
                min_position=config.y_axis.min_position,
                max_position=config.y_axis.max_position,
                axis_offset=config.y_axis.axis_offset,
            )
        else:
            raise NotImplementedError(f'Unknown Y-Axis version: {config.y_axis.version}')

        z_axis: AxisSimulation | TornadoSimulation | SprayerSimulation | None
        if not config.z_axis:
            z_axis = None
        elif isinstance(config.z_axis, ZStepperConfiguration | ZCanOpenConfiguration | AxisD1Configuration):
            z_axis = AxisSimulation(
                min_position=config.z_axis.min_position,
                max_position=config.z_axis.max_position,
                axis_offset=config.z_axis.axis_offset,
            )
        elif isinstance(config.z_axis, TornadoConfiguration):
            z_axis = TornadoSimulation(config=config.z_axis)
        elif isinstance(config.z_axis, SprayerConfiguration):
            z_axis = SprayerSimulation()
        else:
            raise NotImplementedError(f'Unknown Z-Axis version: {config.z_axis.version}')

        flashlight: FlashlightSimulation | FlashlightSimulationV2 | FlashlightPWMSimulationV2 | None
        if not config.flashlight:
            flashlight = None
        elif config.flashlight.version == 'flashlight':
            raise NotImplementedError('This flashlight version is no longer supported.')
        elif config.flashlight.version in ['flashlight_v2', 'flashlight_pwm']:
            flashlight = FlashlightSimulationV2()
        elif config.flashlight.version == 'flashlight_pwm_v2':
            flashlight = FlashlightPWMSimulationV2()
        else:
            raise NotImplementedError(f'Unknown Flashlight version: {config.flashlight.version}')

        estop = rosys.hardware.EStopSimulation()
        bumper = rosys.hardware.BumperSimulation(estop=estop) if config.bumper else None
        bms = rosys.hardware.BmsSimulation()
        imu = rosys.hardware.ImuSimulation(wheels=wheels)
        safety = SafetySimulation(wheels=wheels, estop=estop, y_axis=y_axis,
                                  z_axis=z_axis, flashlight=flashlight)
        modules = [wheels, y_axis, z_axis, flashlight, bumper, imu, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(implement_name=tool,
                         wheels=wheels,
                         flashlight=flashlight,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         imu=imu,
                         bms=bms,
                         safety=safety,
                         modules=active_modules)
