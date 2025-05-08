from dataclasses import dataclass, field
from typing import Literal

from rosys.geometry import Pose, Rectangle, Rotation


@dataclass(kw_only=True)
class RobotBrainConfiguration:
    """Configuration for the robot brain of the FieldFriend robot.

    Defaults:
        enable_esp_on_startup: None
    """
    name: str
    flash_params: list[str]
    enable_esp_on_startup: bool | None = None


@dataclass(kw_only=True)
class MeasurementsConfiguration:
    """Configuration for the measurements of the FieldFriend robot.

    Defaults:
        motor_gear_ratio: 12.52
        wheel_distance: 0.47
        antenna_offset: 0.205
        work_x: 0.0, for dual mechanism, this is work_x_drill
        drill_radius: 0.025
        work_x_chop: None
        chop_radius: None
        work_y: None
    """
    tooth_count: int
    pitch: float
    motor_gear_ratio: float = 12.52
    wheel_distance: float = 0.47
    drill_radius: float = 0.025
    antenna_offset: float = 0.205  # only u1, u2 and f10 are missing this
    work_x: float = 0.0  # this is work_x_drill for dual mechanism
    work_x_chop: float | None = None  # only u2 and u3 have this
    chop_radius: float | None = None  # only u2 and u3 have this
    work_y: float | None = None  # only f16 and f15 have this


@dataclass(kw_only=True)
class CropConfiguration:
    """Configuration for the cropping of the camera of the FieldFriend robot."""
    left: int
    right: int
    up: int
    down: int


@dataclass(kw_only=True)
class CircleSightPositions:
    """Configuration for the positions of the 4 cameras.

    Defaults:
        right = '-1'
        back = '-2'
        front = '-3'
        left = '-4'
    """
    right: str = '-1'
    back: str = '-2'
    front: str = '-3'
    left: str = '-4'


@dataclass(kw_only=True)
class CameraConfiguration:
    """Configuration for the camera of the FieldFriend robot.

    Attributes:
        camera_type: default = 'CalibratableUsbCamera'
        auto_exposure: default = True (not relevant for ZedxminiCamera)
        rotation: default = 0
        fps: default = None, optional (not used for ZedxminiCamera, standard is 10)
        crop: default = None
    """
    width: int
    height: int
    camera_type: Literal['CalibratableUsbCamera', 'ZedxminiCamera'] = 'CalibratableUsbCamera'
    auto_exposure: bool = True  # ZedxminiCamera has no auto exposure, default is True (only True in configs)
    rotation: int = 0
    fps: int | None = None  # standard for CalibratableUsbCamera is 10 (only 10 in configs), ZedxminiCamera has no fps
    crop: CropConfiguration | None = None

    @property
    def crop_rectangle(self) -> Rectangle | None:
        """get a rectangle based on the crop values (left, right, up, down) of the config"""
        if self.crop is None:
            return None
        new_width = self.width - (self.crop.left + self.crop.right)
        new_height = self.height - (self.crop.up + self.crop.down)
        return Rectangle(x=self.crop.left, y=self.crop.up, width=new_width, height=new_height)

    @property
    def parameters(self) -> dict:
        return {
            'width': self.width,
            'height': self.height,
            'auto_exposure': self.auto_exposure,
            'fps': self.fps,
        }


@dataclass(kw_only=True)
class WheelsConfiguration:
    """Configuration for the wheels of the FieldFriend robot.

    Defaults:
        name: 'wheels'
        version: 'double_wheels'
        odrive_version: 4
    """
    is_left_reversed: bool
    is_right_reversed: bool
    left_front_can_address: int
    right_front_can_address: int
    left_back_can_address: int
    right_back_can_address: int
    name: str = 'wheels'
    version: Literal['wheels', 'double_wheels'] = 'double_wheels'
    odrive_version: int = 4
    # left_can_address and right_can_address are only used for wheels and therefore cut, use _front_can_addresses


@dataclass(slots=True, kw_only=True)
class CanConfiguration:
    """Configuration for the can of the FieldFriend robot.

    Defaults:
        name: 'can'
        on_expander: False
        rx_pin: 32
        tx_pin: 33
        baud: 1_000_000
    """
    name: str = 'can'
    on_expander: bool = False
    rx_pin: int = 32
    tx_pin: int = 33
    baud: int = 1_000_000


@dataclass(slots=True, kw_only=True)
class BumperConfiguration:
    """Configuration for the bumper of the FieldFriend robot.

    Defaults:
        name: 'bumper'
        on_expander: True
    """
    pin_front_top: int
    pin_front_bottom: int
    pin_back: int
    name: str = 'bumper'
    on_expander: bool = True

    @property
    def pins(self) -> dict[str, int]:
        return {
            'front_top': self.pin_front_top,
            'front_bottom': self.pin_front_bottom,
            'back': self.pin_back
        }


@dataclass(slots=True, kw_only=True)
class BatteryControlConfiguration:
    """Configuration for the battery control of the FieldFriend robot.

    Defaults:
        name: 'battery_control'
        on_expander: True
        reset_pin: 15
        status_pin: 13
    """
    name: str = 'battery_control'
    on_expander: bool = True
    reset_pin: int = 15
    status_pin: int = 13


@dataclass(slots=True, kw_only=True)
class BmsConfiguration:
    """Configuration for the bms of the FieldFriend robot.

    Defaults:
        name: 'bms'
        on_expander: True
        rx_pin: 26
        tx_pin: 27
        baud: 9600
        num: 2
    """
    name: str = 'bms'
    on_expander: bool = True
    rx_pin: int = 26
    tx_pin: int = 27
    baud: int = 9600
    num: int = 2


@dataclass(slots=True, kw_only=True)
class EstopConfiguration:
    """Configuration for the estop of the FieldFriend robot.

    Defaults:
        name: 'estop'
        pin_1: 34
        pin_2: 35
    """
    name: str = 'estop'
    pin_1: int = 34
    pin_2: int = 35

    @property
    def pins(self) -> dict[str, int]:
        return {
            '1': self.pin_1,
            '2': self.pin_2,
        }


@dataclass(slots=True, kw_only=True)
class FlashlightConfiguration:
    """Configuration for the flashlight of the FieldFriend robot.

    Attributes:
        name: default = 'flashlight'
        version: type of flashlight (v2, pwm, pwm_v2)
        on_expander: bool
        pin: optional int (for pwm)
        rated_voltage: optional float (for pwm)
        front_pin: optional int (for v2, pwm_v2)
        back_pin: optional int (for v2, pwm_v2)
    """
    name: str = 'flashlight'
    # TODO: retire flashlight, only u1 uses it
    # TODO: what about flashlight_v2, only u2 and u3 use it
    version: Literal['flashlight', 'flashlight_v2', 'flashlight_pwm', 'flashlight_pwm_v2']
    on_expander: bool
    pin: int | None = None
    rated_voltage: float | None = None
    front_pin: int | None = None
    back_pin: int | None = None


@dataclass(slots=True, kw_only=True)
class GnssConfiguration:
    """Configuration for the gnss of the FieldFriend robot.

    The yaw should be 90°, but the offset is configured in the septentrio software.

    Defaults:
        antenna_pose: Pose(x=0.041, y=-0.255, yaw=0.0)
    """
    antenna_pose: Pose = field(default_factory=lambda: Pose(x=0.041, y=-0.255, yaw=0.0))


@dataclass(slots=True, kw_only=True)
class ImuConfiguration:
    """Configuration for the IMU of the FieldFriend robot.

    Defaults:
        name: 'imu'
    """
    name: str = 'imu'
    offset_rotation: Rotation = field(default_factory=Rotation.zero)
    min_gyro_calibration: float = 1.0


@dataclass(kw_only=True)
class BaseAxisConfiguration:
    name: str
    max_position: float
    min_position: float
    version: Literal['axis_d1',
                     'chain_axis',
                     'y_axis_stepper',
                     'y_axis_canopen',
                     'tornado',
                     'z_axis_canopen',
                     'z_axis_stepper']


@dataclass(kw_only=True)
class AxisCanOpenConfiguration:
    can_address: int


@dataclass(kw_only=True)
class YAxisConfiguration:
    end_left_pin: int
    end_right_pin: int


@dataclass(kw_only=True)
class StepperPinsConfiguration:
    direction_pin: int
    step_pin: int


@dataclass(kw_only=True)
class OnExpanderConfiguration:
    end_stops_on_expander: bool
    motor_on_expander: bool


@dataclass(kw_only=True)
class StepperAndCanOpenConfiguration:
    end_stops_inverted: bool
    max_speed: int
    reference_speed: int
    steps_per_m: float


@dataclass(kw_only=True)
class EndPinsConfiguration:
    end_bottom_pin: int
    end_top_pin: int


@dataclass(kw_only=True)
class AxisOffsetConfiguration:
    axis_offset: float
    reversed_direction: bool


@dataclass(slots=True, kw_only=True)
class AxisD1Configuration(BaseAxisConfiguration, AxisCanOpenConfiguration, AxisOffsetConfiguration):
    homing_acceleration: int
    homing_velocity: int
    profile_acceleration: int
    profile_velocity: int
    profile_deceleration: int


@dataclass(slots=True, kw_only=True)
class ChainAxisConfiguration(BaseAxisConfiguration,
                             StepperPinsConfiguration,
                             OnExpanderConfiguration,
                             AxisOffsetConfiguration):
    alarm_pin: int
    ref_t_pin: int


@dataclass(slots=True, kw_only=True)
class YStepperConfiguration(BaseAxisConfiguration,
                            YAxisConfiguration,
                            StepperPinsConfiguration,
                            StepperAndCanOpenConfiguration,
                            OnExpanderConfiguration,
                            AxisOffsetConfiguration):
    alarm_inverted: bool
    alarm_pin: int


@dataclass(slots=True, kw_only=True)
class YCanOpenConfiguration(BaseAxisConfiguration,
                            StepperAndCanOpenConfiguration,
                            AxisCanOpenConfiguration,
                            OnExpanderConfiguration,
                            YAxisConfiguration,
                            AxisOffsetConfiguration):
    pass


@dataclass(slots=True, kw_only=True)
class TornadoConfiguration(BaseAxisConfiguration,
                           EndPinsConfiguration,
                           OnExpanderConfiguration):
    """Tornado Configuration

    Defaults:
        current_limit: 30
        end_bottom_pin_expander: False
        is_turn_reversed: True
        is_z_reversed: True
        m_per_tick: 0.025/12.52
        odrive_version: 4
        ref_gear_pin_expander: False
        ref_knife_ground_pin_expander: False
        ref_knife_stop_pin_expander: False
        ref_motor_pin_expander: False
        speed_limit: 1.5
        turn_can_address: 0x400
        z_can_address: 0x500
        turn_reference_speed: 0.25
        turn_speed_limit: 1.5
        z_reference_speed: 0.0075
    """
    ref_gear_pin: int
    ref_knife_ground_pin: int
    ref_knife_stop_pin: int
    ref_motor_pin: int
    current_limit: int = 30
    end_bottom_pin_expander: bool = False
    end_top_pin_expander: bool = False
    is_turn_reversed: bool = True
    is_z_reversed: bool = True
    m_per_tick: float = 0.025/12.52
    odrive_version: int = 4
    ref_gear_pin_expander: bool = False
    ref_knife_ground_pin_expander: bool = False
    ref_knife_stop_pin_expander: bool = False
    ref_motor_pin_expander: bool = False
    end_top_inverted: bool = True
    end_bottom_inverted: bool = True
    ref_motor_inverted: bool = True
    ref_gear_inverted: bool = False
    ref_knife_ground_inverted: bool = False
    ref_knife_stop_inverted: bool = False
    speed_limit: float = 1.5
    turn_can_address: int = 0x400
    z_can_address: int = 0x500
    turn_reference_speed: float = 0.25
    turn_speed_limit: float = 1.5
    z_reference_speed: float = 0.0075


@dataclass(slots=True, kw_only=True)
class ZStepperConfiguration(BaseAxisConfiguration,
                            StepperAndCanOpenConfiguration,
                            EndPinsConfiguration,
                            StepperPinsConfiguration,
                            OnExpanderConfiguration,
                            AxisOffsetConfiguration):
    alarm_pin: int


@dataclass(slots=True, kw_only=True)
class ZCanOpenConfiguration(BaseAxisConfiguration,
                            StepperAndCanOpenConfiguration,
                            AxisCanOpenConfiguration,
                            EndPinsConfiguration,
                            OnExpanderConfiguration,
                            AxisOffsetConfiguration):
    pass


@dataclass(slots=True, kw_only=True)
class FieldFriendConfiguration:
    """Configuration for the FieldFriend robot.

    Defaults:
        can: CanConfiguration
        bms: BmsConfiguration
        estop: EstopConfiguration
        bumper: None
        battery_control: BatteryControlConfiguration
        flashlight: None
    """
    name: str
    robot_brain: RobotBrainConfiguration
    tool: Literal['tornado', 'weed_screw', 'dual_mechanism', 'mower', 'recorder'] | None
    measurements: MeasurementsConfiguration
    wheels: WheelsConfiguration
    has_status_control: bool
    camera: CameraConfiguration | None
    circle_sight_positions: CircleSightPositions | None
    y_axis: AxisD1Configuration | ChainAxisConfiguration | YStepperConfiguration | YCanOpenConfiguration | None
    z_axis: AxisD1Configuration | TornadoConfiguration | ZStepperConfiguration | ZCanOpenConfiguration | None
    can: CanConfiguration = field(default_factory=CanConfiguration)
    bms: BmsConfiguration = field(default_factory=BmsConfiguration)
    estop: EstopConfiguration = field(default_factory=EstopConfiguration)
    bumper: BumperConfiguration | None = None
    battery_control: BatteryControlConfiguration | None = field(default_factory=BatteryControlConfiguration)
    flashlight: FlashlightConfiguration | None = None
    gnss: GnssConfiguration | None = None
    imu: ImuConfiguration | None = None
