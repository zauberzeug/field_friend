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
    DeltaArmConfiguration,
    EstopConfiguration,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    SprayerConfiguration,
    TornadoConfiguration,
    WheelsConfiguration,
    YCanOpenConfiguration,
    YStepperConfiguration,
    ZCanOpenConfiguration,
    ZStepperConfiguration,
    create_drive_parameters,
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
    'DeltaArmConfiguration',
    'EstopConfiguration',
    'FieldFriendConfiguration',
    'FlashlightConfiguration',
    'GnssConfiguration',
    'ImuConfiguration',
    'MeasurementsConfiguration',
    'RobotBrainConfiguration',
    'SprayerConfiguration',
    'TornadoConfiguration',
    'WheelsConfiguration',
    'YCanOpenConfiguration',
    'YStepperConfiguration',
    'ZCanOpenConfiguration',
    'ZStepperConfiguration',
    'create_drive_parameters',
    'get_config',
]
