from rosys.geometry import Pose, Rotation

from field_friend.config.configuration import (
    BumperConfiguration,
    CameraConfiguration,
    CircleSightPositions,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='field-friend-f19',
    robot_brain=RobotBrainConfiguration(name='rb48', flash_params=['orin', 'v05', 'nand'], enable_esp_on_startup=False),
    tool='recorder',
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033, work_x=0.12),
    camera=CameraConfiguration(
        width=1280,
        height=720,
        fps=10,
    ),
    circle_sight_positions=CircleSightPositions(
        right='-1',
        left='-2',
        front='-4',
        back='-3',
    ),
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
    flashlight=FlashlightConfiguration(
        version='flashlight_pwm_v2',
        on_expander=True,
        front_pin=12,
        back_pin=23,
    ),
    bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),

    gnss=GnssConfiguration(antenna_pose=Pose(x=0.041, y=-0.255, yaw=0.0)),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(0.0, 0.0, 0.0)),
)
