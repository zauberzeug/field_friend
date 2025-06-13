from rosys.geometry import Pose, Rotation

from field_friend.config.configuration import (
    BumperConfiguration,
    CameraConfiguration,
    CircleSightPositions,
    CropConfiguration,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='field-friend-f21',
    robot_brain=RobotBrainConfiguration(name='rb54', flash_params=['orin', 'v05', 'nand']),
    tool=None,
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033),
    camera=None,
    wheels=WheelsConfiguration(
        is_left_reversed=True,
        is_right_reversed=True,
        left_back_can_address=0x000,
        left_front_can_address=0x100,
        right_back_can_address=0x200,
        right_front_can_address=0x300,
        odrive_version=6
    ),
    has_status_control=True,
    flashlight=FlashlightConfiguration(
        version='flashlight_pwm_v2',
        front_pin=12,
        back_pin=23,
        on_expander=True,
        rated_voltage=23.0,
    ),
    bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
    y_axis=None,
    z_axis=None,
    circle_sight_positions=CircleSightPositions(
        right='-4',
        left='-3',
        front='-1',
        back='-2',
    ),
    # gnss height: 0.622
    gnss=GnssConfiguration(antenna_pose=Pose(x=0.093, y=0.255, yaw=0.0)),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.6241204, 0.0017964, 0.0)),
)
