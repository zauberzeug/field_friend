import numpy as np
import rosys

# change the config to the config of simulated Robot
import config.config_selection as config_selector

from .chain_axis import ChainAxisSimulation
from .field_friend import FieldFriend
from .flashlight import FlashlightSimulation
from .flashlight_pwm_v2 import FlashlightPWMSimulationV2
from .flashlight_v2 import FlashlightSimulationV2
from .safety import SafetySimulation
from .tornado import TornadoSimulation
from .y_axis import YAxisSimulation
from .y_axis_tornado import YAxisSimulationTornado
from .y_axis_tornado_v2_canopen import YAxisSimulationTornadoV2
from .z_axis import ZAxisSimulation
from .z_axis_v2 import ZAxisSimulationV2


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self, robot_id) -> None:
        config_hardware = config_selector.import_config_simulation(module='hardware', robot_id=robot_id)
        config_params = config_selector.import_config_simulation(module='params', robot_id=robot_id)
        self.MOTOR_GEAR_RATIO = config_params['motor_gear_ratio']
        self.THOOTH_COUNT = config_params['thooth_count']
        self.PITCH = config_params['pitch']
        self.WHEEL_DIAMETER = self.THOOTH_COUNT * self.PITCH / np.pi
        self.M_PER_TICK = self.WHEEL_DIAMETER * np.pi / self.MOTOR_GEAR_RATIO
        self.WHEEL_DISTANCE = config_params['wheel_distance']
        tool = config_params['tool']
        if tool in ['tornado', 'weed_screw', 'none']:
            self.WORK_X = config_params['work_x']
            self.DRILL_RADIUS = config_params['drill_radius']
        elif tool in ['double_mechanism']:
            self.WORK_X_CHOP = config_params['work_x_chop']
            self.WORK_X_DRILL = config_params['work_x_drill']
            self.DRILL_RADIUS = config_params['drill_radius']
            self.CHOP_RADIUS = config_params['chop_radius']
        else:
            raise NotImplementedError(f'Unknown FieldFriend tool: {tool}')
        wheels = rosys.hardware.WheelsSimulation()
        if config_hardware['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisSimulation()
        elif config_hardware['y_axis']['version'] == 'y_axis':
            y_axis = YAxisSimulation()
        elif config_hardware['y_axis']['version'] == 'y_axis_tornado':
            y_axis = YAxisSimulationTornado()
        elif config_hardware['y_axis']['version'] == 'y_axis_tornado_v2':
            y_axis = YAxisSimulationTornadoV2()
        else:
            y_axis = None

        if config_hardware['z_axis']['version'] == 'z_axis':
            z_axis = ZAxisSimulation()
        elif config_hardware['z_axis']['version'] == 'z_axis_v2':
            z_axis = ZAxisSimulationV2(ccw=config_hardware['z_axis']['ccw'])
        elif config_hardware['z_axis']['version'] == 'tornado':
            z_axis = TornadoSimulation(min_position=config_hardware['z_axis']['min_position'],
                                       m_per_tick=config_hardware['z_axis']['m_per_tick'],
                                       is_z_reversed=config_hardware['z_axis']['is_z_reversed'],
                                       is_turn_reversed=config_hardware['z_axis']['is_turn_reversed'])
        else:
            z_axis = None
        if config_hardware['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightSimulation()
        elif config_hardware['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightSimulationV2()
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightSimulationV2()
        elif config_hardware['flashlight']['version'] == 'flashlight_pwm_v2':
            flashlight = FlashlightPWMSimulationV2()
        else:
            flashlight = None

        estop = rosys.hardware.EStopSimulation()
        if 'bumper' in config_hardware:
            bumper = rosys.hardware.BumperSimulation(estop=estop)
        else:
            bumper = None
        bms = rosys.hardware.BmsSimulation()
        safety = SafetySimulation(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [wheels, y_axis, z_axis, flashlight, bumper, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(tool=tool,
                         wheels=wheels,
                         flashlight=flashlight,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
                         safety=safety,
                         modules=active_modules)
