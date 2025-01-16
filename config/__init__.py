from .configuration import (
    AxisD1Configuration,
    BatteryControlConfiguration,
    BmsConfiguration,
    BumperConfiguration,
    CameraConfiguration,
    CameraPositions,
    CanConfiguration,
    ChainAxisConfiguration,
    CropConfiguration,
    EstopConfiguration,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    TornadoConfiguration,
    WheelsConfiguration,
    YCanOpenConfiguration,
    YStepperConfiguration,
    ZCanOpenConfiguration,
    ZStepperConfiguration,
)
from .u4 import u4

robots: dict[str, FieldFriendConfiguration] = {
    'u4': u4
}


def get_config(robot_name: str) -> FieldFriendConfiguration:
    robot_config = robots.get(robot_name)
    if not robot_config:
        raise RuntimeError(f'no configuration found for this robot: {robot_name}')
    return robot_config


__all__ = [
    'AxisD1Configuration',
    'BatteryControlConfiguration',
    'BmsConfiguration',
    'BumperConfiguration',
    'CameraConfiguration',
    'CameraPositions',
    'CanConfiguration',
    'ChainAxisConfiguration',
    'CropConfiguration',
    'EstopConfiguration',
    'FieldFriendConfiguration',
    'FlashlightConfiguration',
    'MeasurementsConfiguration',
    'RobotBrainConfiguration',
    'TornadoConfiguration',
    'WheelsConfiguration',
    'YCanOpenConfiguration',
    'YStepperConfiguration',
    'ZCanOpenConfiguration',
    'ZStepperConfiguration',
    'get_config',
]
