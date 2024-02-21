import rosys
from pyquaternion import Quaternion

from .chain_axis import ChainAxisSimulation
from .configurations import fieldfriend_configurations
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
        if version not in fieldfriend_configurations:
            raise ValueError(f'Unknown FieldFriend version: {version}')
        config = fieldfriend_configurations[version]
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
            z_axis = TornadoSimulation()
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

        imu = rosys.hardware.ImuSimulation(offset_rotation=rosys.geometry.Rotation.zero)
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
                         wheels=wheels,
                         flashlight=flashlight,
                         y_axis=y_axis,
                         z_axis=z_axis,
                         estop=estop,
                         bumper=bumper,
                         bms=bms,
                         safety=safety,
                         imu=imu,
                         modules=active_modules)
