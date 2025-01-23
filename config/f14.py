from .configuration import (
    BumperConfiguration,
    ExternalMowerConfiguration,
    FieldFriendConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='fieldfriend-f14',
    robot_brain=RobotBrainConfiguration(name='rb37', flash_params=['orin', 'v05', 'nand']),
    tool='mower',
    measurements=MeasurementsConfiguration(tooth_count=15, pitch=0.033, work_x=0.47),
    camera=None,
    wheels=WheelsConfiguration(
        is_left_reversed=False,
        is_right_reversed=True,
        left_back_can_address=0x000,
        left_front_can_address=0x100,
        right_back_can_address=0x200,
        right_front_can_address=0x300,
    ),
    has_status_control=True,
    flashlight=None,
    bumper=BumperConfiguration(pin_front_top=18, pin_front_bottom=35, pin_back=21),
    y_axis=None,
    z_axis=None,
    external_mower=ExternalMowerConfiguration(
        m0_can_address=0x400,
        m1_can_address=0x500,
        m2_can_address=0x600,
        speed=10.0,
        odrive_version=6,
    ),
    # TODO: circle sight
    # TODO: gnss
    # TODO: imu
)
