import importlib

from .configuration import (
    AxisD1Configuration,
    BatteryControlConfiguration,
    BmsConfiguration,
    BumperConfiguration,
    CameraConfiguration,
    CanConfiguration,
    ChainAxisConfiguration,
    CircleSightPositions,
    CropConfiguration,
    EstopConfiguration,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    TornadoConfiguration,
    WheelsConfiguration,
    YCanOpenConfiguration,
    YStepperConfiguration,
    ZCanOpenConfiguration,
    ZStepperConfiguration,
)


def get_config(robot_name: str) -> FieldFriendConfiguration:
    try:
        module_name = f'config.{robot_name.lower()}'
        config_module = importlib.import_module(module_name)
        return config_module.config
    except ImportError as e:
        raise RuntimeError(f'No configuration found for robot: {robot_name}') from e


__all__ = [
    'AxisD1Configuration',
    'BatteryControlConfiguration',
    'BmsConfiguration',
    'BumperConfiguration',
    'CameraConfiguration',
    'CanConfiguration',
    'ChainAxisConfiguration',
    'CircleSightPositions',
    'CropConfiguration',
    'EstopConfiguration',
    'FieldFriendConfiguration',
    'FlashlightConfiguration',
    'GnssConfiguration',
    'ImuConfiguration',
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
