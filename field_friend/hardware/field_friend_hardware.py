import logging

import numpy as np
import rosys

from ..config import (
    AxisD1Configuration,
    ChainAxisConfiguration,
    FieldFriendConfiguration,
    TornadoConfiguration,
    YCanOpenConfiguration,
    YStepperConfiguration,
    ZCanOpenConfiguration,
    ZStepperConfiguration,
)
from .axis_d1 import AxisD1
from .can_open_master import CanOpenMasterHardware
from .chain_axis import ChainAxisHardware
from .double_wheels import DoubleWheelsHardware
from .field_friend import FieldFriend
from .flashlight_pwm import FlashlightPWMHardware
from .flashlight_pwm_v2 import FlashlightPWMHardwareV2
from .flashlight_v2 import FlashlightHardwareV2
from .safety import SafetyHardware
from .status_control import StatusControlHardware
from .tornado import TornadoHardware
from .y_axis_canopen_hardware import YAxisCanOpenHardware
from .y_axis_stepper_hardware import YAxisStepperHardware
from .z_axis_canopen_hardware import ZAxisCanOpenHardware
from .z_axis_stepper_hardware import ZAxisStepperHardware


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self, config: FieldFriendConfiguration) -> None:
        self.log = logging.getLogger('field_friend.field_friend_hardware')
        self.MOTOR_GEAR_RATIO: float = config.measurements.motor_gear_ratio
        self.TOOTH_COUNT: int = config.measurements.tooth_count
        self.PITCH: float = config.measurements.pitch
        self.WHEEL_DIAMETER: float = self.TOOTH_COUNT * self.PITCH / np.pi
        self.M_PER_TICK: float = self.WHEEL_DIAMETER * np.pi / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE: float = config.measurements.wheel_distance
        self.ANTENNA_OFFSET: float = config.measurements.antenna_offset
        self.WORK_X: float
        self.DRILL_RADIUS: float
        implement: str | None = config.tool
        if implement in ['tornado', 'weed_screw', None]:
            self.WORK_X = config.measurements.work_x
            if config.measurements.work_y:
                self.WORK_Y: float = config.measurements.work_y
            self.DRILL_RADIUS = config.measurements.drill_radius
        elif implement in ['dual_mechanism']:
            assert config.measurements.work_x_chop
            self.WORK_X_CHOP: float = config.measurements.work_x_chop
            self.WORK_X = config.measurements.work_x
            self.DRILL_RADIUS = config.measurements.drill_radius
            assert config.measurements.chop_radius
            self.CHOP_RADIUS: float = config.measurements.chop_radius
        else:
            self.log.warning('Unknown implement: %s', implement)

        communication = rosys.hardware.SerialCommunication()
        robot_brain = rosys.hardware.RobotBrain(communication,
                                                enable_esp_on_startup=config.robot_brain.enable_esp_on_startup,
                                                use_espresso=config.robot_brain.use_espresso)
        robot_brain.lizard_firmware.flash_params += config.robot_brain.flash_params

        bluetooth = rosys.hardware.BluetoothHardware(robot_brain, name=config.name)
        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        self.can = rosys.hardware.CanHardware(robot_brain,
                                              expander=expander if config.can.on_expander else None,
                                              name=config.can.name,
                                              rx_pin=config.can.rx_pin,
                                              tx_pin=config.can.tx_pin,
                                              baud=config.can.baud)

        wheels: rosys.hardware.WheelsHardware | DoubleWheelsHardware
        if config.wheels.version == 'wheels':
            wheels = rosys.hardware.WheelsHardware(robot_brain,
                                                   can=self.can,
                                                   name=config.wheels.name,
                                                   left_can_address=config.wheels.left_front_can_address,
                                                   right_can_address=config.wheels.right_front_can_address,
                                                   m_per_tick=self.M_PER_TICK,
                                                   width=self.WHEEL_DISTANCE,
                                                   is_right_reversed=config.wheels.is_right_reversed,
                                                   is_left_reversed=config.wheels.is_left_reversed)
        elif config.wheels.version == 'double_wheels':
            wheels = DoubleWheelsHardware(config.wheels, robot_brain, can=self.can,
                                          m_per_tick=self.M_PER_TICK, width=self.WHEEL_DISTANCE)
        else:
            raise NotImplementedError(f'Unknown wheels version: {config.wheels.version}')

        can_open_master = CanOpenMasterHardware(robot_brain, can=self.can, name='master') \
            if ((config.y_axis and config.y_axis.version in ('y_axis_canopen', 'axis_d1')) or
                (config.z_axis and config.z_axis.version in ('z_axis_canopen', 'axis_d1'))) \
            else None

        y_axis: ChainAxisHardware | YAxisStepperHardware | YAxisCanOpenHardware | AxisD1 | None
        if not config.y_axis:
            y_axis = None
        elif isinstance(config.y_axis, AxisD1Configuration) and config.y_axis.version == 'axis_d1':
            y_axis = AxisD1(config.y_axis, robot_brain, can=self.can)
        elif isinstance(config.y_axis, ChainAxisConfiguration) and config.y_axis.version == 'chain_axis':
            y_axis = ChainAxisHardware(config.y_axis, robot_brain, expander=expander)
        elif isinstance(config.y_axis, YStepperConfiguration) and config.y_axis.version == 'y_axis_stepper':
            y_axis = YAxisStepperHardware(config.y_axis, robot_brain, expander=expander)
        elif isinstance(config.y_axis, YCanOpenConfiguration) and config.y_axis.version == 'y_axis_canopen':
            y_axis = YAxisCanOpenHardware(config.y_axis, robot_brain, can=self.can, expander=expander)
        else:
            raise NotImplementedError(f'Unknown y_axis version: {config.y_axis.version}')

        z_axis: TornadoHardware | ZAxisCanOpenHardware | ZAxisStepperHardware | AxisD1 | None
        if not config.z_axis:
            z_axis = None
        elif isinstance(config.z_axis, ZStepperConfiguration) and config.z_axis.version == 'z_axis_stepper':
            z_axis = ZAxisStepperHardware(config.z_axis, robot_brain, expander=expander)
        elif isinstance(config.z_axis, AxisD1Configuration) and config.z_axis.version == 'axis_d1':
            z_axis = AxisD1(config.z_axis, robot_brain, can=self.can)
        elif isinstance(config.z_axis, TornadoConfiguration) and config.z_axis.version == 'tornado':
            z_axis = TornadoHardware(config.z_axis, robot_brain, expander=expander, can=self.can)
        elif isinstance(config.z_axis, ZCanOpenConfiguration) and config.z_axis.version == 'z_axis_canopen':
            z_axis = ZAxisCanOpenHardware(config.z_axis, robot_brain, can=self.can, expander=expander)
        else:
            raise NotImplementedError(f'Unknown z_axis version: {config.z_axis.version}')

        estop = rosys.hardware.EStopHardware(robot_brain, name=config.estop.name, pins=config.estop.pins)

        bms = rosys.hardware.BmsHardware(robot_brain,
                                         expander=expander if config.bms.on_expander else None,
                                         name=config.bms.name,
                                         rx_pin=config.bms.rx_pin,
                                         tx_pin=config.bms.tx_pin,
                                         baud=config.bms.baud,
                                         num=config.bms.num)

        self.battery_control = rosys.hardware.BatteryControlHardware(
            robot_brain,
            expander=expander if config.battery_control.on_expander else None,
            name=config.battery_control.name,
            reset_pin=config.battery_control.reset_pin,
            status_pin=config.battery_control.status_pin,
        ) if config.battery_control else None

        flashlight: FlashlightHardwareV2 | FlashlightPWMHardware | FlashlightPWMHardwareV2 | None
        if not config.flashlight:
            flashlight = None
        elif config.flashlight.version == 'flashlight':
            raise NotImplementedError('This flashlight version is no longer supported.')
        elif config.flashlight.version == 'flashlight_v2':
            assert config.flashlight.front_pin is not None
            assert config.flashlight.back_pin is not None
            flashlight = FlashlightHardwareV2(config.flashlight, robot_brain,
                                              expander=expander if config.flashlight.on_expander else None)
        elif config.flashlight.version == 'flashlight_pwm':
            assert config.flashlight.pin is not None
            assert config.flashlight.rated_voltage is not None
            flashlight = FlashlightPWMHardware(config.flashlight, robot_brain, bms,
                                               expander=expander if config.flashlight.on_expander else None)
        elif config.flashlight.version == 'flashlight_pwm_v2':
            assert config.flashlight.front_pin is not None
            assert config.flashlight.back_pin is not None
            flashlight = FlashlightPWMHardwareV2(config.flashlight, robot_brain, bms,
                                                 expander=expander if config.flashlight.on_expander else None)
        else:
            raise NotImplementedError(f'Unknown Flashlight version: {config.flashlight.version}')

        bumper = rosys.hardware.BumperHardware(robot_brain,
                                               expander=expander if config.bumper.on_expander else None,
                                               estop=estop,
                                               name=config.bumper.name,
                                               pins=config.bumper.pins) if config.bumper else None

        imu = rosys.hardware.ImuHardware(robot_brain,
                                         name=config.imu.name,
                                         offset_rotation=config.imu.offset_rotation,
                                         min_gyro_calibration=config.imu.min_gyro_calibration) if config.imu else None

        self.status_control = StatusControlHardware(robot_brain,
                                                    expander=expander,
                                                    rdyp_pin=39,
                                                    vdp_pin=39) if config.has_status_control else None

        safety: SafetyHardware = SafetyHardware(robot_brain, estop=estop, wheels=wheels, bumper=bumper,
                                                y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)

        modules = [bluetooth, self.can, wheels, serial, expander, can_open_master, y_axis, z_axis,
                   flashlight, bms, estop, self.battery_control, bumper, imu, self.status_control, safety]

        active_modules = [module for module in modules if module is not None]
        super().__init__(implement_name=implement,
                         wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
                         safety=safety,
                         flashlight=flashlight,
                         imu=imu,
                         modules=active_modules,
                         robot_brain=robot_brain)
