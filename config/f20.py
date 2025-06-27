from rosys.geometry import Pose, Rotation

from field_friend.config import (
    BumperConfiguration,
    CircleSightPositions,
    FieldFriendConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='fieldfriend-f20',
    robot_brain=RobotBrainConfiguration(name='rb39', flash_params=['orin', 'v05', 'nand']),
    tool=None,
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033, work_x=0.47),
    camera=None,
    wheels=WheelsConfiguration(
        is_left_reversed=True,
        is_right_reversed=False,
        left_back_can_address=0x000,
        left_front_can_address=0x100,
        right_back_can_address=0x200,
        right_front_can_address=0x300,
        odrive_version=6,
    ),
    has_status_control=True,
    flashlight=None,
    bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
    y_axis=None,
    z_axis=None,
    circle_sight_positions=CircleSightPositions(
        right='-4',
        back='-2',
        front='-1',
        left='-3',
    ),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.6036453, 0.0084839, 0.0), min_gyro_calibration=0.0),
    gnss=GnssConfiguration(antenna_pose=Pose(x=-0.003, y=0.255, yaw=0.0)),
)
