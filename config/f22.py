from rosys.geometry import Pose, Rotation

from field_friend.config.configuration import (
    CameraConfiguration,
    CropConfiguration,
    FieldFriendConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='field-friend-f22',
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
    gnss=GnssConfiguration(antenna_pose=Pose(x=0.041, y=-0.255, yaw=0.0)),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.6006605, 0.0242387, 0.0)),
)
