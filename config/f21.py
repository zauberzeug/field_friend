from field_friend.config.configuration import (
    BumperConfiguration,
    CameraConfiguration,
    CircleSightPositions,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    SprayerConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='fieldfriend-f21',
    robot_brain=RobotBrainConfiguration(name='rb54', flash_params=['orin', 'v05', 'nand'], use_espresso=True),
    tool='sprayer',
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033),
    camera=CameraConfiguration(
        width=1280,
        height=720,
        fps=10,
        rotation=0,
    ),
    wheels=WheelsConfiguration(
        is_left_reversed=False,
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
    z_axis=SprayerConfiguration(),
    circle_sight_positions=CircleSightPositions(
        right='-4',
        left='-3',
        front='-1',
        back='-2',
    ),
    gnss=GnssConfiguration(x=0.093, y=0.255, z=0.665),
    # TODO: IMU configuration is probably wrong. Check https://github.com/zauberzeug/field_friend/pull/361
    # imu=ImuConfiguration(offset_rotation=Rotation.from_euler(-1.6241204, 0.0017964, 0.0)),
    imu=None,
)
