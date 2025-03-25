
import logging

import rosys
from rosys.hardware import ImuSimulation

# change the config to the config of simulated Robot
import config.config_selection as config_selector

from .axis import AxisSimulation
from .chain_axis import ChainAxisSimulation
from .delta_arm import DeltaArmSimulation
from .field_friend import FieldFriend
from .flashlight import FlashlightSimulation
from .flashlight_pwm_v2 import FlashlightPWMSimulationV2
from .flashlight_v2 import FlashlightSimulationV2
from .safety import SafetySimulation
from .tornado import TornadoSimulation


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self, robot_id) -> None:
        self.log = logging.getLogger('field_friend.simulation')
        config_hardware = config_selector.import_config_simulation(module='hardware', robot_id=robot_id)
        config_params = config_selector.import_config_simulation(module='params', robot_id=robot_id)
        self.MOTOR_GEAR_RATIO = config_params['motor_gear_ratio']
        self.THOOTH_COUNT = config_params['thooth_count']
        self.PITCH = config_params['pitch']
        self.WHEEL_DIAMETER = self.THOOTH_COUNT * self.PITCH
        self.M_PER_TICK = self.WHEEL_DIAMETER / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE = config_params['wheel_distance']
        implement: str = config_params['tool']
        if implement in ['tornado', 'weed_screw', 'none']:
            self.WORK_X = config_params['work_x']
            if 'work_y' in config_params:
                self.WORK_Y = config_params['work_y']
            self.DRILL_RADIUS = config_params['drill_radius']
        elif implement in ['dual_mechanism']:
            self.WORK_X_CHOP = config_params['work_x_chop']
            self.WORK_X = config_params['work_x_drill']
            self.DRILL_RADIUS = config_params['drill_radius']
            self.CHOP_RADIUS = config_params['chop_radius']
        else:
            self.log.warning('Unknown implement: %s', implement)
        wheels = rosys.hardware.WheelsSimulation(self.WHEEL_DISTANCE)

        y_axis: AxisSimulation | ChainAxisSimulation | None
        if config_hardware['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisSimulation()
        elif config_hardware['y_axis']['version'] in ['y_axis_stepper', 'y_axis_canopen', 'axis_d1']:
            y_axis = AxisSimulation(
                min_position=config_hardware['y_axis']['min_position'],
                max_position=config_hardware['y_axis']['max_position'],
                axis_offset=config_hardware['y_axis']['axis_offset'],
            )
        elif config_hardware['y_axis']['version'] == 'none':
            y_axis = None
        else:
            raise NotImplementedError(f'Unknown Y-Axis version: {config_hardware["y_axis"]["version"]}')

        z_axis: AxisSimulation | TornadoSimulation | DeltaArmSimulation | None
        if config_hardware['z_axis']['version'] in ['z_axis_stepper', 'z_axis_canopen', 'axis_d1']:
            z_axis = AxisSimulation(
                min_position=config_hardware['z_axis']['min_position'],
                max_position=config_hardware['z_axis']['max_position'],
                axis_offset=config_hardware['z_axis']['axis_offset'],
            )

        elif config_hardware['z_axis']['version'] in ['tornado', 'tornado v1.1']:
            z_axis = TornadoSimulation(min_position=config_hardware['z_axis']['min_position'],
                                       m_per_tick=config_hardware['z_axis']['m_per_tick'],
                                       is_z_reversed=config_hardware['z_axis']['is_z_reversed'],
                                       is_turn_reversed=config_hardware['z_axis']['is_turn_reversed'])
        elif config_hardware['z_axis']['version'] == 'delta_arm':
            z_axis = DeltaArmSimulation()
        elif config_hardware['z_axis']['version'] == 'none':
            z_axis = None
        else:
            raise NotImplementedError(f'Unknown Z-Axis version: {config_hardware["z_axis"]["version"]}')

        flashlight: FlashlightSimulation | FlashlightSimulationV2 | FlashlightPWMSimulationV2 | None
        if config_hardware['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightSimulation()
        elif config_hardware['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightSimulationV2()
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightSimulationV2()
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm_v2':
            flashlight = FlashlightPWMSimulationV2()
        elif config_hardware['flashlight']['version'] == 'none':
            flashlight = None
        else:
            raise NotImplementedError(f'Unknown Flashlight version: {config_hardware["flashlight"]["version"]}')

        estop = rosys.hardware.EStopSimulation()

        bumper: rosys.hardware.BumperSimulation | None = None
        if 'bumper' in config_hardware:
            bumper = rosys.hardware.BumperSimulation(estop=estop)

        imu = ImuSimulation(wheels=wheels, roll_noise=0.0, pitch_noise=0.0, yaw_noise=0.0)
        bms = rosys.hardware.BmsSimulation()
        safety = SafetySimulation(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [wheels, y_axis, z_axis, flashlight, bumper, imu, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(implement_name=implement,
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
