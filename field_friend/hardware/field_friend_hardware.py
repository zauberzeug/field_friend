import numpy as np
import rosys

from .chain_axis import ChainAxisHardware
from .configurations import fieldfriend_configurations
from .double_wheels import DoubleWheelsHardware
from .field_friend import FieldFriend
from .flashlight import FlashlightHardware
from .flashlight_pwm import FlashlightPWMHardware
from .flashlight_v2 import FlashlightHardwareV2
from .imu import IMUHardware
from .safety import SafetyHardware
from .status_control import StatusControlHardware
from .y_axis import YAxisHardware
from .z_axis import ZAxisHardware
from .z_axis_v2 import ZAxisHardwareV2


class FieldFriendHardware(FieldFriend, rosys.hardware.RobotHardware):

    def __init__(self, *, version: str) -> None:
        if version not in fieldfriend_configurations:
            raise NotImplementedError(f'Unknown FieldFriend version: {version}')
        self.config = fieldfriend_configurations[version]
        self.check_pins(self.config)

        self.MOTOR_GEAR_RATIO = self.config['params']['motor_gear_ratio']
        self.THOOTH_COUNT = self.config['params']['thooth_count']
        self.PITCH = self.config['params']['pitch']
        self.WHEEL_DIAMETER = self.THOOTH_COUNT * self.PITCH / np.pi
        self.M_PER_TICK = self.WHEEL_DIAMETER * np.pi / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE = self.config['params']['wheel_distance']
        if version in ['u1', 'ff3', 'u4']:
            self.WORK_X = self.config['params']['work_x']
            self.DRILL_RADIUS = self.config['params']['drill_radius']
        elif version in ['u2', 'u3']:
            self.WORK_X_CHOP = self.config['params']['work_x_chop']
            self.WORK_X_DRILL = self.config['params']['work_x_drill']
            self.DRILL_RADIUS = self.config['params']['drill_radius']
            self.CHOP_RADIUS = self.config['params']['chop_radius']
        else:
            raise NotImplementedError(f'Unknown FieldFriend version: {version}')

        communication = rosys.hardware.SerialCommunication()
        robot_brain = rosys.hardware.RobotBrain(communication)
        # if communication.device_path == '/dev/ttyTHS0':
        #     robot_brain.lizard_firmware.flash_params = ['xavier']
        robot_brain.lizard_firmware.flash_params += self.config['robot_brain']['flash_params']
        bluetooth = rosys.hardware.BluetoothHardware(robot_brain,
                                                     name=self.config['bluetooth']['name'],
                                                     )
        serial = rosys.hardware.SerialHardware(robot_brain)
        expander = rosys.hardware.ExpanderHardware(robot_brain, serial=serial)
        can = rosys.hardware.CanHardware(robot_brain,
                                         expander=expander if self.config['can']['on_expander'] else None,
                                         name=self.config['can']['name'],
                                         rx_pin=self.config['can']['rx_pin'],
                                         tx_pin=self.config['can']['tx_pin'],
                                         baud=self.config['can']['baud'],
                                         )
        if self.config['wheels']['version'] == 'wheels':
            wheels = rosys.hardware.WheelsHardware(robot_brain,
                                                   can=can,
                                                   name=self.config['wheels']['name'],
                                                   left_can_address=self.config['wheels']['left_can_address'],
                                                   right_can_address=self.config['wheels']['right_can_address'],
                                                   m_per_tick=self.M_PER_TICK,
                                                   width=self.WHEEL_DISTANCE,
                                                   is_right_reversed=self.config['wheels']['is_right_reversed'],
                                                   is_left_reversed=self.config['wheels']['is_left_reversed'],
                                                   )
        elif self.config['wheels']['version'] == 'double_wheels':
            wheels = DoubleWheelsHardware(robot_brain,
                                          can=can,
                                          name=self.config['wheels']['name'],
                                          left_back_can_address=self.config['wheels']['left_back_can_address'],
                                          right_back_can_address=self.config['wheels']['right_back_can_address'],
                                          left_front_can_address=self.config['wheels']['left_front_can_address'],
                                          right_front_can_address=self.config['wheels']['right_front_can_address'],
                                          m_per_tick=self.M_PER_TICK,
                                          width=self.WHEEL_DISTANCE,
                                          is_right_reversed=self.config['wheels']['is_right_reversed'],
                                          is_left_reversed=self.config['wheels']['is_left_reversed'],
                                          )
        else:
            raise NotImplementedError(f'Unknown wheels version: {self.config["wheels"]["version"]}')

        if self.config['y_axis']['version'] == 'y_axis':
            y_axis = YAxisHardware(robot_brain,
                                   expander=expander,
                                   name=self.config['y_axis']['name'],
                                   step_pin=self.config['y_axis']['step_pin'],
                                   dir_pin=self.config['y_axis']['dir_pin'],
                                   alarm_pin=self.config['y_axis']['alarm_pin'],
                                   end_l_pin=self.config['y_axis']['end_l_pin'],
                                   end_r_pin=self.config['y_axis']['end_r_pin'],
                                   motor_on_expander=self.config['y_axis']['motor_on_expander'],
                                   end_stops_on_expander=self.config['y_axis']['end_stops_on_expander'],
                                   )
        elif self.config['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisHardware(robot_brain,
                                       expander=expander,
                                       name=self.config['y_axis']['name'],
                                       step_pin=self.config['y_axis']['step_pin'],
                                       dir_pin=self.config['y_axis']['dir_pin'],
                                       alarm_pin=self.config['y_axis']['alarm_pin'],
                                       ref_t_pin=self.config['y_axis']['ref_t_pin'],
                                       motor_on_expander=self.config['y_axis']['motor_on_expander'],
                                       end_stops_on_expander=self.config['y_axis']['end_stops_on_expander'],
                                       )
        else:
            y_axis = None
        if self.config['z_axis']['version'] == 'z_axis':
            z_axis = ZAxisHardware(robot_brain,
                                   expander=expander,
                                   name=self.config['z_axis']['name'],
                                   step_pin=self.config['z_axis']['step_pin'],
                                   dir_pin=self.config['z_axis']['dir_pin'],
                                   alarm_pin=self.config['z_axis']['alarm_pin'],
                                   ref_t_pin=self.config['z_axis']['ref_t_pin'],
                                   end_b_pin=self.config['z_axis']['end_b_pin'],
                                   motor_on_expander=self.config['z_axis']['motor_on_expander'],
                                   end_stops_on_expander=self.config['z_axis']['end_stops_on_expander'],
                                   )
        elif self.config['z_axis']['version'] == 'z_axis_v2':
            z_axis = ZAxisHardwareV2(robot_brain,
                                     expander=expander,
                                     name=self.config['z_axis']['name'],
                                     step_pin=self.config['z_axis']['step_pin'],
                                     dir_pin=self.config['z_axis']['dir_pin'],
                                     alarm_pin=self.config['z_axis']['alarm_pin'],
                                     ref_t_pin=self.config['z_axis']['ref_t_pin'],
                                     end_b_pin=self.config['z_axis']['end_b_pin'],
                                     motor_on_expander=self.config['z_axis']['motor_on_expander'],
                                     end_stops_on_expander=self.config['z_axis']['end_stops_on_expander'],
                                     ref_t_inverted=self.config['z_axis']['ref_t_inverted'],
                                     end_b_inverted=self.config['z_axis']['end_b_inverted'],
                                     ccw=self.config['z_axis']['ccw'],
                                     )
        else:
            z_axis = None
        estop = rosys.hardware.EStopHardware(robot_brain,
                                             name=self.config['estop']['name'],
                                             pins=self.config['estop']['pins'],
                                             )

        bms = rosys.hardware.BmsHardware(robot_brain,
                                         expander=expander if self.config['bms']['on_expander'] else None,
                                         name=self.config['bms']['name'],
                                         rx_pin=self.config['bms']['rx_pin'],
                                         tx_pin=self.config['bms']['tx_pin'],
                                         baud=self.config['bms']['baud'],
                                         num=self.config['bms']['num'],
                                         )
        if 'battery_control' in self.config:
            self.battery_control = rosys.hardware.BatteryControlHardware(
                robot_brain, expander=expander if self.config['battery_control']['on_expander'] else None,
                name=self.config['battery_control']['name'],
                reset_pin=self.config['battery_control']['reset_pin'],
                status_pin=self.config['battery_control']['status_pin'],
            )
        else:
            self.battery_control = None

        if self.config['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightHardware(robot_brain,
                                            expander=expander if self.config['flashlight']['on_expander'] else None,
                                            name=self.config['flashlight']['name'],
                                            pin=self.config['flashlight']['pin'],
                                            )
        elif self.config['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightHardwareV2(robot_brain,
                                              expander=expander if self.config['flashlight']['on_expander'] else None,
                                              name=self.config['flashlight']['name'],
                                              front_pin=self.config['flashlight']['front_pin'],
                                              back_pin=self.config['flashlight']['back_pin'],
                                              )
        elif self.config['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightPWMHardware(robot_brain,
                                               bms,
                                               expander=expander if self.config['flashlight']['on_expander'] else None,
                                               name=self.config['flashlight']['name'],
                                               pin=self.config['flashlight']['pin'],
                                               rated_voltage=self.config['flashlight']['rated_voltage'],
                                               )

        else:
            flashlight = None

        if 'bumper' in self.config:
            bumper = rosys.hardware.BumperHardware(robot_brain,
                                                   expander=expander if self.config['bumper']['on_expander'] else None,
                                                   estop=estop,
                                                   name=self.config['bumper']['name'],
                                                   pins=self.config['bumper']['pins'],
                                                   )
        else:
            bumper = None

        if 'imu' in self.config:
            self.imu = IMUHardware(robot_brain,
                                   name=self.config['imu']['name'],
                                   )
        else:
            self.imu = None

        if 'status_control' in self.config:
            self.status_control = StatusControlHardware(robot_brain,
                                                        expander=expander,
                                                        rdyp_pin=39,
                                                        vdp_pin=39,
                                                        )
        else:
            self.status_control = None

        safety = SafetyHardware(robot_brain, estop=estop, wheels=wheels,
                                y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [bluetooth, can, wheels, serial, expander, y_axis,
                   z_axis, flashlight, bms, estop, self.battery_control, bumper, self.imu, self.status_control, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(version=version,
                         wheels=wheels,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
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
