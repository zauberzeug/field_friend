from rosys.geometry import Pose

from field_friend.config.configuration import (
    BumperConfiguration,
    CameraConfiguration,
    CircleSightPositions,
    FieldFriendConfiguration,
    FlashlightConfiguration,
    GnssConfiguration,
    MeasurementsConfiguration,
    RobotBrainConfiguration,
    WheelsConfiguration,
)

config = FieldFriendConfiguration(
    name='uckerbot-u5',
    robot_brain=RobotBrainConfiguration(name='rb33', flash_params=['orin', 'v05'], enable_esp_on_startup=True, use_espresso=True),
    tool='recorder',
    measurements=MeasurementsConfiguration(
        tooth_count=17,
        pitch=0.041,
        work_x=0.093,
    ),
    camera=CameraConfiguration(
        width=1280,
        height=720,
        fps=10,
    ),
    wheels=WheelsConfiguration(
        is_left_reversed=False,
        is_right_reversed=True,
        left_back_can_address=0x000,
        left_front_can_address=0x100,
        right_back_can_address=0x200,
        right_front_can_address=0x300,
    ),
    has_status_control=True,
    flashlight=FlashlightConfiguration(
        version='flashlight_pwm_v2',
        on_expander=False,
        front_pin=5,
        back_pin=4,
    ),
    bumper=BumperConfiguration(pin_front_top=35, pin_front_bottom=18, pin_back=21),
    y_axis=None,
    z_axis=None,
    circle_sight_positions=CircleSightPositions(),
    gnss=GnssConfiguration(antenna_pose=Pose(x=0.041, y=-0.255, yaw=0.0)),
    # TODO: imu not working with 0.5.2
    imu=None,
)
