import numpy as np
import rosys

from .chain_axis import ChainAxisHardware
from .configurations import fieldfriend_configurations
from .double_wheels import DoubleWheelsHardware
from .field_friend import FieldFriend
from .flashlight import FlashlightHardware
from .flashlight_pwm import FlashlightPWMHardware
from .flashlight_v2 import FlashlightHardwareV2
from .safety import SafetyHardware
from .status_control import StatusControlHardware
from .tornado import TornadoHardware
from .y_axis import YAxisHardware
from .y_axis_tornado import YAxisHardwareTornado
from .z_axis import ZAxisHardware
from .z_axis_v2 import ZAxisHardwareV2


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self, *, version: str) -> None:
        if version not in fieldfriend_configurations:
            raise NotImplementedError(
                f'Unknown FieldFriend version: {version}')
        config: dict[str, dict] = fieldfriend_configurations[version]
        self.check_pins(config)

        self.MOTOR_GEAR_RATIO = config['params']['motor_gear_ratio']
        self.THOOTH_COUNT = config['params']['thooth_count']
        self.PITCH = config['params']['pitch']
        self.WHEEL_DIAMETER = self.THOOTH_COUNT * self.PITCH / np.pi
        self.M_PER_TICK = self.WHEEL_DIAMETER * np.pi / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE = config['params']['wheel_distance']
        tool = config['params']['tool']
        if tool in ['tornado', 'weed_screw', 'none']:
            self.WORK_X = config['params']['work_x']
            self.DRILL_RADIUS = config['params']['drill_radius']
        elif tool in ['double_mechanism']:
            self.WORK_X_CHOP = config['params']['work_x_chop']
            self.WORK_X_DRILL = config['params']['work_x_drill']
            self.DRILL_RADIUS = config['params']['drill_radius']
            self.CHOP_RADIUS = config['params']['chop_radius']
        else:
            raise NotImplementedError(f'Unknown FieldFriend tool: {tool}')
        communication = rosys.hardware.SerialCommunication()
        robot_brain = rosys.hardware.RobotBrain(communication)
        # if communication.device_path == '/dev/ttyTHS0':
        #     robot_brain.lizard_firmware.flash_params = ['xavier']
        robot_brain.lizard_firmware.flash_params += config['robot_brain']['flash_params']
        bluetooth = rosys.hardware.BluetoothHardware(robot_brain,
                                                     name=config['bluetooth']['name'],
                                                     )
        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        can = rosys.hardware.CanHardware(robot_brain,
                                         expander=expander if config['can']['on_expander'] else None,
                                         name=config['can']['name'],
                                         rx_pin=config['can']['rx_pin'],
                                         tx_pin=config['can']['tx_pin'],
                                         baud=config['can']['baud'],
                                         )
        if config['wheels']['version'] == 'wheels':
            wheels = rosys.hardware.WheelsHardware(robot_brain,
                                                   can=can,
                                                   name=config['wheels']['name'],
                                                   left_can_address=config['wheels']['left_can_address'],
                                                   right_can_address=config['wheels']['right_can_address'],
                                                   m_per_tick=self.M_PER_TICK,
                                                   width=self.WHEEL_DISTANCE,
                                                   is_right_reversed=config['wheels']['is_right_reversed'],
                                                   is_left_reversed=config['wheels']['is_left_reversed'],
                                                   )
        elif config['wheels']['version'] == 'double_wheels':
            wheels = DoubleWheelsHardware(robot_brain,
                                          can=can,
                                          name=config['wheels']['name'],
                                          left_back_can_address=config['wheels']['left_back_can_address'],
                                          right_back_can_address=config['wheels']['right_back_can_address'],
                                          left_front_can_address=config['wheels']['left_front_can_address'],
                                          right_front_can_address=config['wheels']['right_front_can_address'],
                                          m_per_tick=self.M_PER_TICK,
                                          width=self.WHEEL_DISTANCE,
                                          is_right_reversed=config['wheels']['is_right_reversed'],
                                          is_left_reversed=config['wheels']['is_left_reversed'],
                                          )
        else:
            raise NotImplementedError(f'Unknown wheels version: {config["wheels"]["version"]}')

        if config['y_axis']['version'] == 'y_axis':
            y_axis = YAxisHardware(robot_brain,
                                   expander=expander,
                                   name=config['y_axis']['name'],
                                   step_pin=config['y_axis']['step_pin'],
                                   dir_pin=config['y_axis']['dir_pin'],
                                   alarm_pin=config['y_axis']['alarm_pin'],
                                   end_l_pin=config['y_axis']['end_l_pin'],
                                   end_r_pin=config['y_axis']['end_r_pin'],
                                   motor_on_expander=config['y_axis']['motor_on_expander'],
                                   end_stops_on_expander=config['y_axis']['end_stops_on_expander'],
                                   )
        elif config['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisHardware(robot_brain,
                                       expander=expander,
                                       name=config['y_axis']['name'],
                                       step_pin=config['y_axis']['step_pin'],
                                       dir_pin=config['y_axis']['dir_pin'],
                                       alarm_pin=config['y_axis']['alarm_pin'],
                                       ref_t_pin=config['y_axis']['ref_t_pin'],
                                       motor_on_expander=config['y_axis']['motor_on_expander'],
                                       end_stops_on_expander=config['y_axis']['end_stops_on_expander'],
                                       )
        elif config['y_axis']['version'] == 'y_axis_tornado':
            y_axis = YAxisHardwareTornado(robot_brain,
                                          expander=expander,
                                          name=config['y_axis']['name'],
                                          max_speed=config['y_axis']['max_speed'],
                                          min_position=config['y_axis']['min_position'],
                                          max_position=config['y_axis']['max_position'],
                                          axis_offset=config['y_axis']['axis_offset'],
                                          steps_per_m=config['y_axis']['steps_per_m'],
                                          step_pin=config['y_axis']['step_pin'],
                                          dir_pin=config['y_axis']['dir_pin'],
                                          alarm_pin=config['y_axis']['alarm_pin'],
                                          end_r_pin=config['y_axis']['end_r_pin'],
                                          end_l_pin=config['y_axis']['end_l_pin'],
                                          motor_on_expander=config['y_axis']['motor_on_expander'],
                                          end_stops_on_expander=config['y_axis']['end_stops_on_expander'],
                                          )
        else:
            y_axis = None
        if config['z_axis']['version'] == 'z_axis':
            z_axis = ZAxisHardware(robot_brain,
                                   expander=expander,
                                   name=config['z_axis']['name'],
                                   step_pin=config['z_axis']['step_pin'],
                                   dir_pin=config['z_axis']['dir_pin'],
                                   alarm_pin=config['z_axis']['alarm_pin'],
                                   ref_t_pin=config['z_axis']['ref_t_pin'],
                                   end_b_pin=config['z_axis']['end_b_pin'],
                                   motor_on_expander=config['z_axis']['motor_on_expander'],
                                   end_stops_on_expander=config['z_axis']['end_stops_on_expander'],
                                   )
        elif config['z_axis']['version'] == 'z_axis_v2':
            z_axis = ZAxisHardwareV2(robot_brain,
                                     expander=expander,
                                     name=config['z_axis']['name'],
                                     step_pin=config['z_axis']['step_pin'],
                                     dir_pin=config['z_axis']['dir_pin'],
                                     alarm_pin=config['z_axis']['alarm_pin'],
                                     ref_t_pin=config['z_axis']['ref_t_pin'],
                                     end_b_pin=config['z_axis']['end_b_pin'],
                                     motor_on_expander=config['z_axis']['motor_on_expander'],
                                     end_stops_on_expander=config['z_axis']['end_stops_on_expander'],
                                     ref_t_inverted=config['z_axis']['ref_t_inverted'],
                                     end_b_inverted=config['z_axis']['end_b_inverted'],
                                     ccw=config['z_axis']['ccw'],
                                     )
        elif config['z_axis']['version'] == 'tornado':
            z_axis = TornadoHardware(robot_brain,
                                     expander=expander,
                                     can=can,
                                     name=config['z_axis']['name'],
                                     min_position=config['z_axis']['min_position'],
                                     z_can_address=config['z_axis']['z_can_address'],
                                     turn_can_address=config['z_axis']['turn_can_address'],
                                     m_per_tick=config['z_axis']['m_per_tick'],
                                     end_top_pin=config['z_axis']['end_top_pin'],
                                     end_bottom_pin=config['z_axis']['end_bottom_pin'],
                                     ref_motor_pin=config['z_axis']['ref_motor_pin'],
                                     ref_gear_pin=config['z_axis']['ref_gear_pin'],
                                     ref_t_pin=config['z_axis']['ref_t_pin'],
                                     ref_b_pin=config['z_axis']['ref_b_pin'],
                                     motors_on_expander=config['z_axis']['motors_on_expander'],
                                     end_stops_on_expander=config['z_axis']['end_stops_on_expander'],
                                     is_z_reversed=config['z_axis']['is_z_reversed'],
                                     is_turn_reversed=config['z_axis']['is_turn_reversed'],
                                     speed_limit=config['z_axis']['speed_limit'],
                                     turn_speed_limit=config['z_axis']['turn_speed_limit'],
                                     current_limit=config['z_axis']['current_limit'],
                                     )
        else:
            z_axis = None
        estop = rosys.hardware.EStopHardware(robot_brain,
                                             name=config['estop']['name'],
                                             pins=config['estop']['pins'],
                                             )

        bms = rosys.hardware.BmsHardware(robot_brain,
                                         expander=expander if config['bms']['on_expander'] else None,
                                         name=config['bms']['name'],
                                         rx_pin=config['bms']['rx_pin'],
                                         tx_pin=config['bms']['tx_pin'],
                                         baud=config['bms']['baud'],
                                         num=config['bms']['num'],
                                         )
        if 'battery_control' in config:
            self.battery_control = rosys.hardware.BatteryControlHardware(
                robot_brain, expander=expander if config['battery_control']['on_expander'] else None,
                name=config['battery_control']['name'],
                reset_pin=config['battery_control']['reset_pin'],
                status_pin=config['battery_control']['status_pin'],
            )
        else:
            self.battery_control = None

        if config['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightHardware(robot_brain,
                                            expander=expander if config['flashlight']['on_expander'] else None,
                                            name=config['flashlight']['name'],
                                            pin=config['flashlight']['pin'],
                                            )
        elif config['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightHardwareV2(robot_brain,
                                              expander=expander if config['flashlight']['on_expander'] else None,
                                              name=config['flashlight']['name'],
                                              front_pin=config['flashlight']['front_pin'],
                                              back_pin=config['flashlight']['back_pin'],
                                              )
        elif config['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightPWMHardware(robot_brain,
                                               bms,
                                               expander=expander if config['flashlight']['on_expander'] else None,
                                               name=config['flashlight']['name'],
                                               pin=config['flashlight']['pin'],
                                               rated_voltage=config['flashlight']['rated_voltage'],
                                               )

        else:
            flashlight = None

        if 'bumper' in config:
            bumper = rosys.hardware.BumperHardware(robot_brain,
                                                   expander=expander if config['bumper']['on_expander'] else None,
                                                   estop=estop,
                                                   name=config['bumper']['name'],
                                                   pins=config['bumper']['pins'],
                                                   )
        else:
            bumper = None

        if 'imu' in self.config:
            imu = rosys.hardware.ImuHardware(robot_brain=robot_brain,
                                             name=self.config['imu']['name'],
                                             offset_rotation=rosys.geometry.Rotation.from_euler(
                                                 np.radians(self.config['imu']['roll_offset']),
                                                 np.radians(self.config['imu']['pitch_offset']),
                                                 np.radians(self.config['imu']['yaw_offset'])))
        else:
            imu = None

        if 'status_control' in config:
            self.status_control = StatusControlHardware(robot_brain,
                                                        expander=expander,
                                                        rdyp_pin=39,
                                                        vdp_pin=39,
                                                        )
        else:
            self.status_control = None

        safety = SafetyHardware(robot_brain, estop=estop, wheels=wheels, bumper=bumper,
                                y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [bluetooth, can, wheels, serial, expander, y_axis,
                   z_axis, flashlight, bms, estop, self.battery_control, bumper, imu, self.status_control, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(version=version,
                         tool=tool,
                         wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
                         imu=imu,
                         safety=safety,
                         flashlight=flashlight,
                         modules=active_modules,
                         robot_brain=robot_brain)

    def check_pins(self, d):
        for key, value in d.items():
            if isinstance(value, dict):
                self.check_pins(value)  # Recurse into sub-dict
            elif (key.endswith('pin') or key in ['boot', 'enable']) and value == 6:
                raise ValueError(f'Pin 6 is used in {key}')
