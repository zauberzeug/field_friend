import numpy as np
import rosys

# change the config to the config of simulated Robot
from config.config_rb28 import fieldfriend_configuration

from .chain_axis import ChainAxisSimulation
from .field_friend import FieldFriend
from .flashlight import FlashlightSimulation
from .flashlight_v2 import FlashlightSimulationV2
from .safety import SafetySimulation
from .tornado import TornadoSimulation
from .y_axis import YAxisSimulation
from .y_axis_tornado import YAxisSimulationTornado
from .z_axis import ZAxisSimulation
from .z_axis_v2 import ZAxisSimulationV2


class FieldFriendSimulation(FieldFriend, rosys.hardware.RobotSimulation):

    def __init__(self,  version: str) -> None:
        config: dict[str, dict] = fieldfriend_configuration

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
        wheels = rosys.hardware.WheelsSimulation()
        if config['y_axis']['version'] == 'chain_axis':
            y_axis = ChainAxisSimulation()
        elif config['y_axis']['version'] == 'y_axis':
            y_axis = YAxisSimulation()
        elif config['y_axis']['version'] == 'y_axis_tornado':
            y_axis = YAxisSimulationTornado()
        else:
            y_axis = None

        if config['z_axis']['version'] == 'z_axis':
            z_axis = ZAxisSimulation()
        elif config['z_axis']['version'] == 'z_axis_v2':
            z_axis = ZAxisSimulationV2(ccw=config['z_axis']['ccw'])
        elif config['z_axis']['version'] == 'tornado':
            z_axis = TornadoSimulation(min_position=config['z_axis']['min_position'],
                                       m_per_tick=config['z_axis']['m_per_tick'],
                                       is_z_reversed=config['z_axis']['is_z_reversed'],
                                       is_turn_reversed=config['z_axis']['is_turn_reversed'])
        else:
            z_axis = None
        if config['flashlight']['version'] == 'flashlight':
            flashlight = FlashlightSimulation()
        elif config['flashlight']['version'] == 'flashlight_v2':
            flashlight = FlashlightSimulationV2()
        elif config['flashlight']['version'] == 'flashlight_pwm':
            flashlight = FlashlightSimulationV2()
        else:
            flashlight = None

        estop = rosys.hardware.EStopSimulation()
        if 'bumper' in config:
            bumper = rosys.hardware.BumperSimulation(estop=estop)
        else:
            bumper = None
        bms = rosys.hardware.BmsSimulation()
        safety = SafetySimulation(wheels=wheels, estop=estop, y_axis=y_axis, z_axis=z_axis, flashlight=flashlight)
        modules = [wheels, y_axis, z_axis, flashlight, bumper, bms, estop, safety]
        active_modules = [module for module in modules if module is not None]
        super().__init__(version=version,
                         tool=tool,
                         wheels=wheels,
                         flashlight=flashlight,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
                         safety=safety,
                         modules=active_modules)
