import rosys
from rosys.hardware import ImuHardware

import config.config_selection as config_selector

from .axis_d1 import AxisD1
from .can_open_master import CanOpenMasterHardware
from .chain_axis import ChainAxisHardware
from .double_wheels import DoubleWheelsHardware
from .external_mower import MowerHardware
from .field_friend import FieldFriend
from .flashlight import FlashlightHardware
from .flashlight_pwm import FlashlightPWMHardware
from .flashlight_pwm_v2 import FlashlightPWMHardwareV2
from .flashlight_v2 import FlashlightHardwareV2
from .led_eyes import LedEyesHardware
from .safety import SafetyHardware
from .status_control import StatusControlHardware
from .tornado import TornadoHardware
from .y_axis_canopen_hardware import YAxisCanOpenHardware
from .y_axis_stepper_hardware import YAxisStepperHardware
from .z_axis_canopen_hardware import ZAxisCanOpenHardware
from .z_axis_stepper_hardware import ZAxisStepperHardware


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self) -> None:
        config_hardware: dict = config_selector.import_config(module='hardware')
        config_robotbrain: dict = config_selector.import_config(module='robotbrain')
        config_params: dict = config_selector.import_config(module='params')
        self.check_pins(config_hardware)
        self.MOTOR_GEAR_RATIO: float = config_params['motor_gear_ratio']
        self.THOOTH_COUNT: int = config_params['thooth_count']
        self.PITCH: float = config_params['pitch']
        self.WHEEL_DIAMETER: float = self.THOOTH_COUNT * self.PITCH
        self.M_PER_TICK: float = self.WHEEL_DIAMETER / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE: float = config_params['wheel_distance']
        self.WORK_X: float
        self.DRILL_RADIUS: float
        implement: str = config_params['tool']
        if implement in ['tornado', 'weed_screw', 'none']:
            self.WORK_X = config_params['work_x']
            if 'work_y' in config_params:
                self.WORK_Y: float = config_params['work_y']
            self.DRILL_RADIUS = config_params['drill_radius']
        elif implement in ['dual_mechanism']:
            self.WORK_X_CHOP: float = config_params['work_x_chop']
            self.WORK_X = config_params['work_x_drill']
            self.DRILL_RADIUS = config_params['drill_radius']
            self.CHOP_RADIUS: float = config_params['chop_radius']
        elif implement in ['mower']:  # front mower for trees
            self.WORK_X = 0.0
            self.DRILL_RADIUS = 0.0
        else:
            raise NotImplementedError(f'Unknown FieldFriend implement: {implement}')

        communication = rosys.hardware.SerialCommunication()
        if 'enable_esp_on_startup' in config_robotbrain['robot_brain']:
            robot_brain = rosys.hardware.RobotBrain(
                communication, enable_esp_on_startup=config_robotbrain['robot_brain']['enable_esp_on_startup'])
        else:
            robot_brain = rosys.hardware.RobotBrain(communication)

        robot_brain.lizard_firmware.flash_params += config_robotbrain['robot_brain']['flash_params']
        bluetooth = rosys.hardware.BluetoothHardware(
            robot_brain,
            name=config_hardware['bluetooth']['name'],
        )
        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        can = rosys.hardware.CanHardware(
            robot_brain,
            expander=expander if config_hardware['can']['on_expander'] else None,
            name=config_hardware['can']['name'],
            rx_pin=config_hardware['can']['rx_pin'],
            tx_pin=config_hardware['can']['tx_pin'],
            baud=config_hardware['can']['baud'],
        )
        wheels: rosys.hardware.WheelsHardware | DoubleWheelsHardware
        if config_hardware['wheels']['version'] == 'wheels':
            wheels = rosys.hardware.WheelsHardware(
                robot_brain,
                can=can,
                name=config_hardware['wheels']['name'],
                left_can_address=config_hardware['wheels']['left_can_address'],
                right_can_address=config_hardware['wheels']['right_can_address'],
                m_per_tick=self.M_PER_TICK,
                width=self.WHEEL_DISTANCE,
                is_right_reversed=config_hardware['wheels']['is_right_reversed'],
                is_left_reversed=config_hardware['wheels']['is_left_reversed'],
            )
        elif config_hardware['wheels']['version'] == 'double_wheels':
            wheels = DoubleWheelsHardware(
                robot_brain,
                can=can,
                name=config_hardware['wheels']['name'],
                left_back_can_address=config_hardware['wheels']['left_back_can_address'],
                right_back_can_address=config_hardware['wheels']['right_back_can_address'],
                left_front_can_address=config_hardware['wheels']['left_front_can_address'],
                right_front_can_address=config_hardware['wheels']['right_front_can_address'],
                m_per_tick=self.M_PER_TICK,
                width=self.WHEEL_DISTANCE,
                is_right_reversed=config_hardware['wheels']['is_right_reversed'],
                is_left_reversed=config_hardware['wheels']['is_left_reversed'],
                odrive_version=config_hardware['wheels']['odrive_version']if 'odrive_version' in config_hardware['wheels'] else 4,
            )
        else:
            raise NotImplementedError(f'Unknown wheels version: {config_hardware["wheels"]["version"]}')

        if config_hardware['y_axis']['version'] in ('y_axis_canopen', 'axis_d1') or config_hardware['z_axis']['version'] in ('y_axis_canopen', 'axis_d1'):
            can_open_master = CanOpenMasterHardware(robot_brain, can=can, name='master')
        else:
            can_open_master = None
        y_axis: ChainAxisHardware | YAxisStepperHardware | YAxisCanOpenHardware | AxisD1 | None
        if config_hardware['y_axis']['version'] == 'axis_d1':
            y_axis = AxisD1(
                robot_brain,
                can=can,
                can_address=config_hardware['y_axis']['can_address'],
                name=config_hardware['y_axis']['name'],
                homing_acceleration=config_hardware['y_axis']['homing_acceleration'],
                homing_velocity=config_hardware['y_axis']['homing_velocity'],
                profile_acceleration=config_hardware['y_axis']['profile_acceleration'],
                profile_velocity=config_hardware['y_axis']['profile_velocity'],
                profile_deceleration=config_hardware['y_axis']['profile_deceleration'],
                max_position=config_hardware['y_axis']['max_position'],
                min_position=config_hardware['y_axis']['min_position'],
                axis_offset=config_hardware['y_axis']['axis_offset'],
                reverse_direction=config_hardware['y_axis']['reversed_direction'],)
        elif config_hardware['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisHardware(
                robot_brain,
                expander=expander,
                name=config_hardware['y_axis']['name'],
                min_position=config_hardware['y_axis']['min_position'],
                max_position=config_hardware['y_axis']['max_position'],
                step_pin=config_hardware['y_axis']['step_pin'],
                dir_pin=config_hardware['y_axis']['dir_pin'],
                alarm_pin=config_hardware['y_axis']['alarm_pin'],
                ref_t_pin=config_hardware['y_axis']['ref_t_pin'],
                motor_on_expander=config_hardware['y_axis']['motor_on_expander'],
                end_stops_on_expander=config_hardware['y_axis']['end_stops_on_expander'],
            )
        elif config_hardware['y_axis']['version'] == 'y_axis_stepper':
            try:
                alarm_inverted: bool = config_hardware['y_axis']['alarm_inverted']
            except KeyError as e:
                raise KeyError("'alarm_inverted' not found in config_hardware['y_axis']. Robots with a "
                               'Closed-Loop-Step-Direction-Motor (U4 and before) has an inverted motor alarm input. '
                               "Check your robot's setup.") from e
            y_axis = YAxisStepperHardware(
                robot_brain,
                expander=expander,
                name=config_hardware['y_axis']['name'],
                max_speed=config_hardware['y_axis']['max_speed'],
                reference_speed=config_hardware['y_axis']['reference_speed'],
                min_position=config_hardware['y_axis']['min_position'],
                max_position=config_hardware['y_axis']['max_position'],
                axis_offset=config_hardware['y_axis']['axis_offset'],
                steps_per_m=config_hardware['y_axis']['steps_per_m'],
                step_pin=config_hardware['y_axis']['step_pin'],
                dir_pin=config_hardware['y_axis']['dir_pin'],
                alarm_pin=config_hardware['y_axis']['alarm_pin'],
                alarm_inverted=alarm_inverted,
                end_r_pin=config_hardware['y_axis']['end_r_pin'],
                end_l_pin=config_hardware['y_axis']['end_l_pin'],
                motor_on_expander=config_hardware['y_axis']['motor_on_expander'],
                end_stops_on_expander=config_hardware['y_axis']['end_stops_on_expander'],
                reversed_direction=config_hardware['y_axis']['reversed_direction'],
                end_stops_inverted=config_hardware['y_axis']['end_stops_inverted'],
            )
        elif config_hardware['y_axis']['version'] == 'y_axis_canopen':
            y_axis = YAxisCanOpenHardware(
                robot_brain,
                can=can,
                can_address=config_hardware['y_axis']['can_address'],
                expander=expander,
                name=config_hardware['y_axis']['name'],
                max_speed=config_hardware['y_axis']['max_speed'],
                reference_speed=config_hardware['y_axis']['reference_speed'],
                min_position=config_hardware['y_axis']['min_position'],
                max_position=config_hardware['y_axis']['max_position'],
                axis_offset=config_hardware['y_axis']['axis_offset'],
                steps_per_m=config_hardware['y_axis']['steps_per_m'],
                end_l_pin=config_hardware['y_axis']['end_l_pin'],
                end_r_pin=config_hardware['y_axis']['end_r_pin'],
                motor_on_expander=config_hardware['y_axis']['motor_on_expander'],
                end_stops_on_expander=config_hardware['y_axis']['end_stops_on_expander'],
                reversed_direction=config_hardware['y_axis']['reversed_direction'],
                end_stops_inverted=config_hardware['y_axis']['end_stops_inverted'],
            )
        elif config_hardware['y_axis']['version'] == 'none':
            y_axis = None
        else:
            raise NotImplementedError(f'Unknown y_axis version: {config_hardware["y_axis"]["version"]}')

        z_axis: TornadoHardware | ZAxisCanOpenHardware | ZAxisStepperHardware | AxisD1 | None
        if config_hardware['z_axis']['version'] == 'z_axis_stepper':
            z_axis = ZAxisStepperHardware(
                robot_brain,
                expander=expander,
                name=config_hardware['z_axis']['name'],
                max_speed=config_hardware['z_axis']['max_speed'],
                reference_speed=config_hardware['z_axis']['reference_speed'],
                min_position=config_hardware['z_axis']['min_position'],
                max_position=config_hardware['z_axis']['max_position'],
                axis_offset=config_hardware['z_axis']['axis_offset'],
                steps_per_m=config_hardware['z_axis']['steps_per_m'],
                step_pin=config_hardware['z_axis']['step_pin'],
                dir_pin=config_hardware['z_axis']['dir_pin'],
                alarm_pin=config_hardware['z_axis']['alarm_pin'],
                end_t_pin=config_hardware['z_axis']['end_t_pin'],
                end_b_pin=config_hardware['z_axis']['end_b_pin'],
                motor_on_expander=config_hardware['z_axis']['motor_on_expander'],
                end_stops_on_expander=config_hardware['z_axis']['end_stops_on_expander'],
                reversed_direction=config_hardware['z_axis']['reversed_direction'],
                end_stops_inverted=config_hardware['z_axis']['end_stops_inverted'],
            )
        elif config_hardware['z_axis']['version'] == 'axis_d1':
            z_axis = AxisD1(
                robot_brain,
                can=can,
                can_address=config_hardware['z_axis']['can_address'],
                name=config_hardware['z_axis']['name'],
                homing_acceleration=config_hardware['z_axis']['homing_acceleration'],
                homing_velocity=config_hardware['z_axis']['homing_velocity'],
                profile_acceleration=config_hardware['z_axis']['profile_acceleration'],
                profile_velocity=config_hardware['z_axis']['profile_velocity'],
                profile_deceleration=config_hardware['z_axis']['profile_deceleration'],
                max_position=config_hardware['z_axis']['max_position'],
                min_position=config_hardware['z_axis']['min_position'],
                axis_offset=config_hardware['z_axis']['axis_offset'],
                reverse_direction=config_hardware['z_axis']['reversed_direction'],)
        elif config_hardware['z_axis']['version'] == 'tornado':
            z_axis = TornadoHardware(
                robot_brain,
                expander=expander,
                can=can,
                name=config_hardware['z_axis']['name'],
                min_position=config_hardware['z_axis']['min_position'],
                z_can_address=config_hardware['z_axis']['z_can_address'],
                turn_can_address=config_hardware['z_axis']['turn_can_address'],
                m_per_tick=config_hardware['z_axis']['m_per_tick'],
                end_top_pin=config_hardware['z_axis']['end_top_pin'],
                end_bottom_pin=config_hardware['z_axis']['end_bottom_pin'],
                ref_motor_pin=config_hardware['z_axis']['ref_motor_pin'],
                ref_gear_pin=config_hardware['z_axis']['ref_gear_pin'],
                ref_knife_stop_pin=config_hardware['z_axis']['ref_knife_stop_pin'],
                ref_knife_ground_pin=config_hardware['z_axis']['ref_knife_ground_pin'],
                motors_on_expander=config_hardware['z_axis']['motors_on_expander'],
                end_stops_on_expander=config_hardware['z_axis']['end_stops_on_expander'],
                is_z_reversed=config_hardware['z_axis']['is_z_reversed'],
                is_turn_reversed=config_hardware['z_axis']['is_turn_reversed'],
                speed_limit=config_hardware['z_axis']['speed_limit'],
                turn_speed_limit=config_hardware['z_axis']['turn_speed_limit'],
                current_limit=config_hardware['z_axis']['current_limit'],
                z_reference_speed=config_hardware['z_axis']['z_reference_speed'],
                turn_reference_speed=config_hardware['z_axis']['turn_reference_speed'],
                odrive_version=config_hardware['z_axis']['odrive_version']if 'odrive_version' in config_hardware['z_axis'] else 4,
            )
        elif config_hardware['z_axis']['version'] == 'tornado v1.1':
            z_axis = TornadoHardware(
                robot_brain,
                expander=expander,
                can=can,
                name=config_hardware['z_axis']['name'],
                min_position=config_hardware['z_axis']['min_position'],
                z_can_address=config_hardware['z_axis']['z_can_address'],
                turn_can_address=config_hardware['z_axis']['turn_can_address'],
                m_per_tick=config_hardware['z_axis']['m_per_tick'],
                end_top_pin=config_hardware['z_axis']['end_top_pin'],
                end_top_pin_expander=config_hardware['z_axis']['end_top_pin_expander'],
                end_bottom_pin=config_hardware['z_axis']['end_bottom_pin'],
                end_bottom_pin_expander=config_hardware['z_axis']['end_bottom_pin_expander'],
                ref_motor_pin=config_hardware['z_axis']['ref_motor_pin'],
                ref_gear_pin=config_hardware['z_axis']['ref_gear_pin'],
                ref_gear_pin_expander=config_hardware['z_axis']['ref_gear_pin_expander'],
                ref_knife_stop_pin=config_hardware['z_axis']['ref_knife_stop_pin'],
                ref_knife_stop_pin_expander=config_hardware['z_axis']['ref_knife_stop_pin_expander'],
                ref_knife_ground_pin=config_hardware['z_axis']['ref_knife_ground_pin'],
                ref_knife_ground_pin_expander=config_hardware['z_axis']['ref_knife_ground_pin_expander'],
                motors_on_expander=config_hardware['z_axis']['motors_on_expander'],
                end_stops_on_expander=config_hardware['z_axis']['end_stops_on_expander'],
                is_z_reversed=config_hardware['z_axis']['is_z_reversed'],
                is_turn_reversed=config_hardware['z_axis']['is_turn_reversed'],
                speed_limit=config_hardware['z_axis']['speed_limit'],
                turn_speed_limit=config_hardware['z_axis']['turn_speed_limit'],
                current_limit=config_hardware['z_axis']['current_limit'],
                z_reference_speed=config_hardware['z_axis']['z_reference_speed'],
                turn_reference_speed=config_hardware['z_axis']['turn_reference_speed'],
                odrive_version=config_hardware['z_axis']['odrive_version']if 'odrive_version' in config_hardware['z_axis'] else 4,
            )
        elif config_hardware['z_axis']['version'] == 'z_axis_canopen':
            z_axis = ZAxisCanOpenHardware(
                robot_brain,
                can=can,
                can_address=config_hardware['z_axis']['can_address'],
                expander=expander,
                name=config_hardware['z_axis']['name'],
                max_speed=config_hardware['z_axis']['max_speed'],
                reference_speed=config_hardware['z_axis']['reference_speed'],
                min_position=config_hardware['z_axis']['min_position'],
                max_position=config_hardware['z_axis']['max_position'],
                axis_offset=config_hardware['z_axis']['axis_offset'],
                steps_per_m=config_hardware['z_axis']['steps_per_m'],
                end_t_pin=config_hardware['z_axis']['end_t_pin'],
                end_b_pin=config_hardware['z_axis']['end_b_pin'],
                motor_on_expander=config_hardware['z_axis']['motor_on_expander'],
                end_stops_on_expander=config_hardware['z_axis']['end_stops_on_expander'],
                reversed_direction=config_hardware['z_axis']['reversed_direction'],
                end_stops_inverted=config_hardware['z_axis']['end_stops_inverted'],
            )
        elif config_hardware['z_axis']['version'] == 'none':
            z_axis = None
        else:
            raise NotImplementedError(f'Unknown z_axis version: {config_hardware["z_axis"]["version"]}')

        mower: MowerHardware | None
        if 'external_mower' in config_hardware:
            mower = MowerHardware(
                robot_brain=robot_brain,
                can=can,
                name=config_hardware['external_mower']['name'],
                m0_can_address=config_hardware['external_mower']['m0_can_address'],
                m1_can_address=config_hardware['external_mower']['m1_can_address'],
                m2_can_address=config_hardware['external_mower']['m2_can_address'],
                m_per_tick=self.M_PER_TICK,
                speed=config_hardware['external_mower']['speed'],
                is_m0_reversed=config_hardware['external_mower']['is_m0_reversed'],
                is_m1_reversed=config_hardware['external_mower']['is_m1_reversed'],
                is_m2_reversed=config_hardware['external_mower']['is_m2_reversed'],
                odrive_version=config_hardware['external_mower'][
                    'odrive_version'] if 'odrive_version' in config_hardware['external_mower'] else 4,
            )
        else:
            mower = None

        estop = rosys.hardware.EStopHardware(
            robot_brain,
            name=config_hardware['estop']['name'],
            pins=config_hardware['estop']['pins'],
        )

        bms = rosys.hardware.BmsHardware(
            robot_brain,
            expander=expander if config_hardware['bms']['on_expander'] else None,
            name=config_hardware['bms']['name'],
            rx_pin=config_hardware['bms']['rx_pin'],
            tx_pin=config_hardware['bms']['tx_pin'],
            baud=config_hardware['bms']['baud'],
            num=config_hardware['bms']['num'],
        )
        self.battery_control: rosys.hardware.BatteryControlHardware | None
        if 'battery_control' in config_hardware:
            self.battery_control = rosys.hardware.BatteryControlHardware(
                robot_brain, expander=expander if config_hardware['battery_control']['on_expander'] else None,
                name=config_hardware['battery_control']['name'],
                reset_pin=config_hardware['battery_control']['reset_pin'],
                status_pin=config_hardware['battery_control']['status_pin'],
            )
        else:
            self.battery_control = None

        flashlight: FlashlightHardware | FlashlightHardwareV2 | FlashlightPWMHardware | FlashlightPWMHardwareV2 | None
        if config_hardware['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightHardware(
                robot_brain,
                expander=expander if config_hardware['flashlight']['on_expander'] else None,
                name=config_hardware['flashlight']['name'],
                pin=config_hardware['flashlight']['pin'],
            )
        elif config_hardware['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightHardwareV2(
                robot_brain,
                expander=expander if config_hardware['flashlight']['on_expander'] else None,
                name=config_hardware['flashlight']['name'],
                front_pin=config_hardware['flashlight']['front_pin'],
                back_pin=config_hardware['flashlight']['back_pin'],
            )
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightPWMHardware(
                robot_brain,
                bms,
                expander=expander if config_hardware['flashlight']['on_expander'] else None,
                name=config_hardware['flashlight']['name'],
                pin=config_hardware['flashlight']['pin'],
                rated_voltage=config_hardware['flashlight']['rated_voltage'],
            )
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm_v2':
            flashlight = FlashlightPWMHardwareV2(
                robot_brain,
                bms,
                expander=expander if config_hardware['flashlight']['on_expander'] else None,
                name=config_hardware['flashlight']['name'],
                front_pin=config_hardware['flashlight']['front_pin'],
                back_pin=config_hardware['flashlight']['back_pin'],
            )
        elif config_hardware['flashlight']['version'] == 'none':
            flashlight = None
        else:
            raise NotImplementedError(f'Unknown Flashlight version: {config_hardware["flashlight"]["version"]}')

        bumper: rosys.hardware.BumperHardware | None
        if 'bumper' in config_hardware:
            bumper = rosys.hardware.BumperHardware(
                robot_brain,
                expander=expander if config_hardware['bumper']['on_expander'] else None,
                estop=estop,
                name=config_hardware['bumper']['name'],
                pins=config_hardware['bumper']['pins'],
            )
        else:
            bumper = None

        imu: rosys.hardware.Imu | None = None
        if 'imu' in config_hardware:
            imu = ImuHardware(
                robot_brain=robot_brain,
                name=config_hardware['imu']['name'],
                offset_rotation=rosys.geometry.Rotation.from_euler(
                    *config_hardware['imu']['offset_rotation']),
                min_gyro_calibration=config_hardware['imu'].get('min_gyro_calibration', 0.0)
            )

        eyes: LedEyesHardware | None
        if 'eyes' in config_hardware:
            eyes = LedEyesHardware(
                robot_brain,
                expander=expander if config_hardware['eyes']['on_expander'] else None,
                name=config_hardware['eyes']['name'],
                eyes_pin=config_hardware['eyes']['eyes_pin'],
            )
        else:
            eyes = None

        self.status_control: StatusControlHardware | None
        if 'status_control' in config_hardware:
            self.status_control = StatusControlHardware(
                robot_brain,
                expander=expander,
                rdyp_pin=39,
                vdp_pin=39,
            )
        else:
            self.status_control = None

        safety: SafetyHardware = SafetyHardware(robot_brain, estop=estop, wheels=wheels, bumper=bumper,
                                                y_axis=y_axis, z_axis=z_axis, flashlight=flashlight, mower=mower)

        modules = [bluetooth, can, wheels, serial, expander, can_open_master, y_axis,
                   z_axis, mower, flashlight, bms, estop, self.battery_control, bumper, imu, eyes, self.status_control, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(
            implement_name=implement,
            wheels=wheels,
            y_axis=y_axis,
            z_axis=z_axis,
            estop=estop,
            bumper=bumper,
            bms=bms,
            safety=safety,
            flashlight=flashlight,
            mower=mower,
            imu=imu,
            modules=active_modules,
            robot_brain=robot_brain
        )

    def check_pins(self, d):
        for key, value in d.items():
            if isinstance(value, dict):
                self.check_pins(value)  # Recurse into sub-dict
            elif (key.endswith('pin') or key in ['boot', 'enable']) and value == 6:
                raise ValueError(f'Pin 6 is used in {key}')
