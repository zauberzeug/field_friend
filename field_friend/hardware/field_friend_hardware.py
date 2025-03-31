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
        if config.robot_brain.enable_esp_on_startup is not None:
            robot_brain = rosys.hardware.RobotBrain(communication,
                                                    enable_esp_on_startup=config.robot_brain.enable_esp_on_startup)
        else:
            robot_brain = rosys.hardware.RobotBrain(communication)

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
            wheels = DoubleWheelsHardware(robot_brain,
                                          can=self.can,
                                          name=config.wheels.name,
                                          left_back_can_address=config.wheels.left_back_can_address,
                                          right_back_can_address=config.wheels.right_back_can_address,
                                          left_front_can_address=config.wheels.left_front_can_address,
                                          right_front_can_address=config.wheels.right_front_can_address,
                                          m_per_tick=self.M_PER_TICK,
                                          width=self.WHEEL_DISTANCE,
                                          is_right_reversed=config.wheels.is_right_reversed,
                                          is_left_reversed=config.wheels.is_left_reversed,
                                          odrive_version=config.wheels.odrive_version)
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
            y_axis = AxisD1(robot_brain,
                            can=self.can,
                            can_address=config.y_axis.can_address,
                            name=config.y_axis.name,
                            homing_acceleration=config.y_axis.homing_acceleration,
                            homing_velocity=config.y_axis.homing_velocity,
                            profile_acceleration=config.y_axis.profile_acceleration,
                            profile_velocity=config.y_axis.profile_velocity,
                            profile_deceleration=config.y_axis.profile_deceleration,
                            max_position=config.y_axis.max_position,
                            min_position=config.y_axis.min_position,
                            axis_offset=config.y_axis.axis_offset,
                            reverse_direction=config.y_axis.reversed_direction,
                            )
        elif isinstance(config.y_axis, ChainAxisConfiguration) and config.y_axis.version == 'chain_axis':
            y_axis = ChainAxisHardware(robot_brain,
                                       expander=expander,
                                       name=config.y_axis.name,
                                       min_position=config.y_axis.min_position,
                                       max_position=config.y_axis.max_position,
                                       step_pin=config.y_axis.step_pin,
                                       dir_pin=config.y_axis.dir_pin,
                                       alarm_pin=config.y_axis.alarm_pin,
                                       ref_t_pin=config.y_axis.ref_t_pin,
                                       motor_on_expander=config.y_axis.motor_on_expander,
                                       end_stops_on_expander=config.y_axis.end_stops_on_expander,
                                       )
        elif isinstance(config.y_axis, YStepperConfiguration) and config.y_axis.version == 'y_axis_stepper':
            y_axis = YAxisStepperHardware(robot_brain,
                                          expander=expander,
                                          name=config.y_axis.name,
                                          max_speed=config.y_axis.max_speed,
                                          reference_speed=config.y_axis.reference_speed,
                                          min_position=config.y_axis.min_position,
                                          max_position=config.y_axis.max_position,
                                          axis_offset=config.y_axis.axis_offset,
                                          steps_per_m=config.y_axis.steps_per_m,
                                          step_pin=config.y_axis.step_pin,
                                          dir_pin=config.y_axis.dir_pin,
                                          alarm_pin=config.y_axis.alarm_pin,
                                          alarm_inverted=config.y_axis.alarm_inverted,
                                          end_r_pin=config.y_axis.end_r_pin,
                                          end_l_pin=config.y_axis.end_l_pin,
                                          motor_on_expander=config.y_axis.motor_on_expander,
                                          end_stops_on_expander=config.y_axis.end_stops_on_expander,
                                          reversed_direction=config.y_axis.reversed_direction,
                                          end_stops_inverted=config.y_axis.end_stops_inverted,
                                          )
        elif isinstance(config.y_axis, YCanOpenConfiguration) and config.y_axis.version == 'y_axis_canopen':
            y_axis = YAxisCanOpenHardware(robot_brain,
                                          can=self.can,
                                          can_address=config.y_axis.can_address,
                                          expander=expander,
                                          name=config.y_axis.name,
                                          max_speed=config.y_axis.max_speed,
                                          reference_speed=config.y_axis.reference_speed,
                                          min_position=config.y_axis.min_position,
                                          max_position=config.y_axis.max_position,
                                          axis_offset=config.y_axis.axis_offset,
                                          steps_per_m=config.y_axis.steps_per_m,
                                          end_l_pin=config.y_axis.end_l_pin,
                                          end_r_pin=config.y_axis.end_r_pin,
                                          motor_on_expander=config.y_axis.motor_on_expander,
                                          end_stops_on_expander=config.y_axis.end_stops_on_expander,
                                          reversed_direction=config.y_axis.reversed_direction,
                                          end_stops_inverted=config.y_axis.end_stops_inverted,
                                          )
        else:
            raise NotImplementedError(f'Unknown y_axis version: {config.y_axis.version}')

        z_axis: TornadoHardware | ZAxisCanOpenHardware | ZAxisStepperHardware | AxisD1 | None
        if not config.z_axis:
            z_axis = None
        elif isinstance(config.z_axis, ZStepperConfiguration) and config.z_axis.version == 'z_axis_stepper':
            z_axis = ZAxisStepperHardware(robot_brain,
                                          expander=expander,
                                          name=config.z_axis.name,
                                          max_speed=config.z_axis.max_speed,
                                          reference_speed=config.z_axis.reference_speed,
                                          min_position=config.z_axis.min_position,
                                          max_position=config.z_axis.max_position,
                                          axis_offset=config.z_axis.axis_offset,
                                          steps_per_m=config.z_axis.steps_per_m,
                                          step_pin=config.z_axis.step_pin,
                                          dir_pin=config.z_axis.dir_pin,
                                          alarm_pin=config.z_axis.alarm_pin,
                                          end_t_pin=config.z_axis.end_top_pin,
                                          end_b_pin=config.z_axis.end_bottom_pin,
                                          motor_on_expander=config.z_axis.motor_on_expander,
                                          end_stops_on_expander=config.z_axis.end_stops_on_expander,
                                          reversed_direction=config.z_axis.reversed_direction,
                                          end_stops_inverted=config.z_axis.end_stops_inverted,
                                          )
        elif isinstance(config.z_axis, AxisD1Configuration) and config.z_axis.version == 'axis_d1':
            z_axis = AxisD1(robot_brain,
                            can=self.can,
                            can_address=config.z_axis.can_address,
                            name=config.z_axis.name,
                            homing_acceleration=config.z_axis.homing_acceleration,
                            homing_velocity=config.z_axis.homing_velocity,
                            profile_acceleration=config.z_axis.profile_acceleration,
                            profile_velocity=config.z_axis.profile_velocity,
                            profile_deceleration=config.z_axis.profile_deceleration,
                            max_position=config.z_axis.max_position,
                            min_position=config.z_axis.min_position,
                            axis_offset=config.z_axis.axis_offset,
                            reverse_direction=config.z_axis.reversed_direction,
                            )
        elif isinstance(config.z_axis, TornadoConfiguration) and config.z_axis.version == 'tornado':
            z_axis = TornadoHardware(robot_brain,
                                     expander=expander,
                                     can=self.can,
                                     name=config.z_axis.name,
                                     min_position=config.z_axis.min_position,
                                     z_can_address=config.z_axis.z_can_address,
                                     turn_can_address=config.z_axis.turn_can_address,
                                     m_per_tick=config.z_axis.m_per_tick,
                                     end_top_pin=config.z_axis.end_top_pin,
                                     end_top_pin_expander=config.z_axis.end_top_pin_expander,
                                     end_bottom_pin=config.z_axis.end_bottom_pin,
                                     end_bottom_pin_expander=config.z_axis.end_bottom_pin_expander,
                                     ref_motor_pin=config.z_axis.ref_motor_pin,
                                     ref_gear_pin=config.z_axis.ref_gear_pin,
                                     ref_gear_pin_expander=config.z_axis.ref_gear_pin_expander,
                                     ref_knife_stop_pin=config.z_axis.ref_knife_stop_pin,
                                     ref_knife_stop_pin_expander=config.z_axis.ref_knife_stop_pin_expander,
                                     ref_knife_ground_pin=config.z_axis.ref_knife_ground_pin,
                                     ref_knife_ground_pin_expander=config.z_axis.ref_knife_ground_pin_expander,
                                     motors_on_expander=config.z_axis.motor_on_expander,
                                     end_stops_on_expander=config.z_axis.end_stops_on_expander,
                                     is_z_reversed=config.z_axis.is_z_reversed,
                                     is_turn_reversed=config.z_axis.is_turn_reversed,
                                     speed_limit=config.z_axis.speed_limit,
                                     turn_speed_limit=config.z_axis.turn_speed_limit,
                                     current_limit=config.z_axis.current_limit,
                                     z_reference_speed=config.z_axis.z_reference_speed,
                                     turn_reference_speed=config.z_axis.turn_reference_speed,
                                     odrive_version=config.z_axis.odrive_version,
                                     )
        elif isinstance(config.z_axis, ZCanOpenConfiguration) and config.z_axis.version == 'z_axis_canopen':
            z_axis = ZAxisCanOpenHardware(robot_brain,
                                          can=self.can,
                                          can_address=config.z_axis.can_address,
                                          expander=expander,
                                          name=config.z_axis.name,
                                          max_speed=config.z_axis.max_speed,
                                          reference_speed=config.z_axis.reference_speed,
                                          min_position=config.z_axis.min_position,
                                          max_position=config.z_axis.max_position,
                                          axis_offset=config.z_axis.axis_offset,
                                          steps_per_m=config.z_axis.steps_per_m,
                                          end_t_pin=config.z_axis.end_top_pin,
                                          end_b_pin=config.z_axis.end_bottom_pin,
                                          motor_on_expander=config.z_axis.motor_on_expander,
                                          end_stops_on_expander=config.z_axis.end_stops_on_expander,
                                          reversed_direction=config.z_axis.reversed_direction,
                                          end_stops_inverted=config.z_axis.end_stops_inverted,
                                          )
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
            flashlight = FlashlightHardwareV2(robot_brain,
                                              expander=expander if config.flashlight.on_expander else None,
                                              name=config.flashlight.name,
                                              front_pin=config.flashlight.front_pin,
                                              back_pin=config.flashlight.back_pin,
                                              )
        elif config.flashlight.version == 'flashlight_pwm':
            assert config.flashlight.pin is not None
            assert config.flashlight.rated_voltage is not None
            flashlight = FlashlightPWMHardware(robot_brain,
                                               bms,
                                               expander=expander if config.flashlight.on_expander else None,
                                               name=config.flashlight.name,
                                               pin=config.flashlight.pin,
                                               rated_voltage=config.flashlight.rated_voltage,
                                               )
        elif config.flashlight.version == 'flashlight_pwm_v2':
            assert config.flashlight.front_pin is not None
            assert config.flashlight.back_pin is not None
            flashlight = FlashlightPWMHardwareV2(robot_brain,
                                                 bms,
                                                 expander=expander if config.flashlight.on_expander else None,
                                                 name=config.flashlight.name,
                                                 front_pin=config.flashlight.front_pin,
                                                 back_pin=config.flashlight.back_pin,
                                                 )
        else:
            raise NotImplementedError(f'Unknown Flashlight version: {config.flashlight.version}')

        bumper = rosys.hardware.BumperHardware(robot_brain,
                                               expander=expander if config.bumper.on_expander else None,
                                               estop=estop,
                                               name=config.bumper.name,
                                               pins=config.bumper.pins) if config.bumper else None

        imu = rosys.hardware.ImuHardware(robot_brain,
                                         name=config.imu.name,
                                         offset_rotation=config.imu.offset_rotation) if config.imu else None

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
