from rosys.geometry import Rotation

from field_friend.config.configuration import (  # BumperConfiguration,
    CameraConfiguration,
    CircleSightPositions,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    ImuConfiguration,
    MeasurementsConfiguration,
    MowerConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='fieldfriend-f23',
    robot_brain=RobotBrainConfiguration(name='rb57', flash_params=['orin', 'v05', 'nand'], use_espresso=True),
    tool=None,
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033),
    camera=CameraConfiguration(
        width=1280,
        height=720,
        fps=10,
        rotation=0,
    ),
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
    flashlight=FlashlightConfiguration(
        version='flashlight_pwm_v2',
        front_pin=12,
        back_pin=23,
        on_expander=True,
        rated_voltage=23.0,
    ),
    # bumper=BumperConfiguration(pin_front_top=21, pin_front_bottom=35, pin_back=18),
    bumper=None,
    y_axis=None,
    z_axis=MowerConfiguration(),
    circle_sight_positions=CircleSightPositions(),
    gnss=GnssConfiguration(),
    imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.603092, 0.020933, 1.570120), min_gyro_calibration=0.0),
)
