from rosys.geometry import Rotation

from field_friend.config.configuration import (
    FieldFriendConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='fieldfriend-f22',
    robot_brain=RobotBrainConfiguration(name='rb55', flash_params=['orin', 'v05', 'nand']),
    tool=None,
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033, work_x=0, wheel_distance=1.49),
    camera=None,
    wheels=WheelsConfiguration(
        is_left_reversed=True,
        is_right_reversed=False,
        left_back_can_address=0x000,
        left_front_can_address=0x100,
        right_back_can_address=0x200,
        right_front_can_address=0x300,
        odrive_version=6
    ),
    has_status_control=True,
    flashlight=None,
    bumper=None,
    y_axis=None,
    z_axis=None,
    circle_sight_positions=None,
    # TODO: height
    gnss=GnssConfiguration(x=0.0, y=0.778),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.5804, 0.00506, 0.0)),
)
