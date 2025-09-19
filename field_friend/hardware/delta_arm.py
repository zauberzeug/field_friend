from abc import ABC, abstractmethod

import numpy as np
import rosys
from nicegui import ui
from rosys.geometry import Frame3d, Point, Point3d, Rotation, axes_object
from rosys.helpers import remove_indentation

from ..config import DeltaArmConfiguration


class DeltaArm(rosys.hardware.Module, ABC):
    def __init__(self, config: DeltaArmConfiguration):
        super().__init__()
        self.config = config
        self.l1 = self.config.l1
        self.l2 = self.config.l2
        self.b = self.config.b
        self.p = self.config.p
        self.height = self.config.height
        self.y_limit = getattr(self.config, 'y_limit', None)

        self.angle_offset_left = np.deg2rad(0)
        self.angle_offset_right = np.deg2rad(0)

        self.base_frame = Frame3d(id='delta_robot.base', x=-0.15, z=self.height)
        self.servo_frame_left = Frame3d(id='delta_robot.servo_left', y=self.b) \
            .in_frame(self.base_frame)
        self.l1_left_frame = Frame3d(id='delta_robot.l1_left', y=self.l1, rotation=Rotation.from_euler(0, 0, np.pi)) \
            .in_frame(self.servo_frame_left)

        self.servo_frame_right = Frame3d(id='delta_robot.servo_right', y=-self.b, rotation=Rotation.from_euler(0, 0, np.pi)) \
            .in_frame(self.base_frame)
        self.l1_right_frame = Frame3d(id='delta_robot.l1_right', y=self.l1, rotation=Rotation.from_euler(0, 0, np.pi)) \
            .in_frame(self.servo_frame_right)

        self.end_effector_frame = Frame3d(id='delta_robot.end_effector', z=-0.270) \
            .in_frame(self.base_frame)

        self.l2_right_frame = Frame3d(id='delta_robot.l2_right', y=-self.p) \
            .in_frame(self.end_effector_frame)
        self.l2_left_frame = Frame3d(id='delta_robot.l2_left', y=self.p) \
            .in_frame(self.end_effector_frame)

        rosys.on_startup(self.reset)

    @property
    def tool_position(self) -> tuple[float, float]:
        return projection_yz(self.end_effector_frame.relative_to(self.base_frame).point_3d).tuple

    def move_to_angles(self, theta_left: float, theta_right: float) -> bool:
        self.servo_frame_left.rotation = Rotation.from_euler(theta_left + self.angle_offset_left, 0, 0)
        self.servo_frame_right.rotation = Rotation.from_euler(theta_right + self.angle_offset_right, 0, 0)
        l2_angle_right = projection_yz(self.l2_right_frame.relative_to(self.base_frame).point_3d) \
            .direction(projection_yz(self.l1_right_frame.relative_to(self.base_frame).point_3d))
        self.l2_right_frame.rotation = Rotation.from_euler(l2_angle_right, 0, 0)

        l2_angle_left = projection_yz(self.l2_left_frame.relative_to(self.base_frame).point_3d) \
            .direction(projection_yz(self.l1_left_frame.relative_to(self.base_frame).point_3d))
        self.l2_left_frame.rotation = Rotation.from_euler(l2_angle_left, 0, 0)
        return True

    @abstractmethod
    def move_to_position(self, y: float, z: float) -> bool:
        pass

    def move_relative(self, y: float, z: float) -> bool:
        return self.move_to_position(self.tool_position[0] + y, self.tool_position[1] + z)

    async def stop(self) -> None:
        pass

    def reset(self) -> None:
        success = self.move_to_position(0, -0.270)
        if not success:
            raise RuntimeError('DeltaArm failed to move to reset position (y=0, z=-0.270)')

    def calculate_angles(self, y: float, z: float) -> tuple[float, float]:
        """
        Calculate the motor angles required to position the end effector at (y, z)
        Args:
            y: Y-axis coordinate (left/right) of the desired position
            z: Z-axis coordinate (up/down) of the desired position
        Returns:
            tuple: (theta_left, theta_right) angles in radians for the two motors
        """
        # Adjust for the platform width (end effector joints are at ±p)
        y_left = y + self.p   # Left arm target joint
        y_right = y - self.p  # Right arm target joint
        # Calculate for left arm (motor 1)
        # Distance from base joint to target
        d_left = np.sqrt((y_left - self.b)**2 + z**2)
        # Use law of cosines to find the angle
        cos_alpha_left = (self.l1**2 + d_left**2 - self.l2**2) / (2 * self.l1 * d_left)
        # Clamp to valid range to avoid numerical errors
        cos_alpha_left = np.clip(cos_alpha_left, -1.0, 1.0)
        alpha_left = np.arccos(cos_alpha_left)
        # Calculate the base angle
        beta_left = np.arctan2(z, y_left - self.b)
        # The final angle for motor 1
        theta_left = beta_left + alpha_left
        # Calculate for right arm (motor 2)
        # Distance from base joint to target
        d_right = np.sqrt((y_right + self.b)**2 + z**2)
        # Use law of cosines to find the angle
        cos_alpha_right = (self.l1**2 + d_right**2 - self.l2**2) / (2 * self.l1 * d_right)
        # Clamp to valid range to avoid numerical errors
        cos_alpha_right = np.clip(cos_alpha_right, -1.0, 1.0)
        alpha_right = np.arccos(cos_alpha_right)
        # Calculate the base angle
        beta_right = np.arctan2(z, y_right + self.b)
        # The final angle for motor 2
        theta_right = beta_right - alpha_right
        return theta_left, theta_right

    def is_position_reachable(self, y: float, z: float) -> bool:
        """
        Check if a position (y, z) is reachable by the delta robot.
        Args:
            y: Y-axis coordinate to check
            z: Z-axis coordinate to check
        Returns:
            bool: True if the position is reachable, False otherwise
        """
        # Adjust for the platform width (end effector joints at ±p, servos at ±b)
        y_left = y + self.p
        y_right = y - self.p
        # Distances from servo joints (at +b and -b) to target joints
        d_left = np.sqrt((y_left - self.b)**2 + z**2)
        d_right = np.sqrt((y_right + self.b)**2 + z**2)
        # Check if the distances are within the range that can be reached
        # The minimum distance is |L1 - L2| and the maximum is L1 + L2
        min_reach = abs(self.l1 - self.l2)
        max_reach = self.l1 + self.l2
        within_reach = (
            min_reach <= d_left <= max_reach and
            min_reach <= d_right <= max_reach and
            -self.height <= z <= 0
        )
        if not within_reach:
            return False
        if self.y_limit is not None and not (-self.y_limit <= y <= self.y_limit):
            return False
        return True

    def check_angles(self, theta_left, theta_right):
        theta_left = rosys.helpers.eliminate_2pi(theta_left)
        theta_right = rosys.helpers.eliminate_2pi(theta_right)
        self.log.debug(f'Checking angles: {np.rad2deg(theta_left)}, {np.rad2deg(theta_right)}')
        return -np.pi <= theta_left <= np.pi and -np.pi <= theta_right <= np.pi

    def developer_ui(self) -> None:
        ui.label('Info').classes('text-center text-bold')
        with ui.row():
            ui.label('Tool Position')
            ui.label('').bind_text_from(self, 'tool_position', lambda x: f'{x[0]:.3f} m')
            ui.label('').bind_text_from(self, 'tool_position', lambda x: f'{x[1]:.3f} m')
            ui.label('').bind_text_from(self.servo_frame_left, 'rotation',
                                        lambda rotation: f'Theta Left: {np.rad2deg(rosys.helpers.angle(rotation.roll, self.angle_offset_left)):.3f} deg')
            ui.label('').bind_text_from(self.servo_frame_right, 'rotation',
                                        lambda rotation: f'Theta Right: {np.rad2deg(rosys.helpers.angle(rotation.roll, self.angle_offset_right)):.3f} deg')

        ui.label('Delta Control').classes('text-center text-bold')
        with ui.row():
            ui.button('Reset', on_click=self.reset)

        ui.separator()
        ui.label('Absolute Position').classes('text-center text-bold')
        with ui.row():
            y_input = ui.number(label='Y', value=self.tool_position[0], format='%.3f', suffix='m')
            z_input = ui.number(label='Z', value=self.tool_position[1], format='%.3f', suffix='m')
        ui.button('Move', on_click=lambda: self.move_to_position(y_input.value, z_input.value))
        ui.separator()
        ui.label('Relative Position').classes('text-center text-bold')
        step_input = ui.number(label='Stepsize', value=0.005, format='%.3f', suffix='m')
        with ui.row():
            ui.button(icon='chevron_left', on_click=lambda: self.move_relative(step_input.value, 0))
            with ui.column():
                ui.button(icon='expand_less', on_click=lambda: self.move_relative(0, step_input.value))
                ui.button(icon='expand_more', on_click=lambda: self.move_relative(0, -step_input.value))
            ui.button(icon='chevron_right', on_click=lambda: self.move_relative(-step_input.value, 0))

    def scene_object(self) -> None:
        axes_object(self.base_frame, name='Base Frame', show_x=False, length=self.b)
        axes_object(self.l1_left_frame, name='L1 Left', show_x=False, show_z=False, length=self.l1)
        axes_object(self.l1_right_frame, name='L1 Right', show_x=False, show_z=False, length=self.l1)
        axes_object(self.l2_left_frame, name='L2 Left', show_x=False, show_z=False, length=self.l2)
        axes_object(self.l2_right_frame, name='L2 Right', show_x=False, show_z=False, length=self.l2)
        axes_object(self.end_effector_frame, name='End Effector', length=self.p)


class DeltaArmHardware(DeltaArm, rosys.hardware.ModuleHardware):
    # TODO: Implement hardware interface
    def __init__(self, config: DeltaArmConfiguration, robot_brain: rosys.hardware.RobotBrain, can: rosys.hardware.CanHardware, *, name: str = 'delta_arm', **kwargs) -> None:
        self.name = name
        lizard_code: str = remove_indentation('')
        core_message_fields: list[str] = []
        rosys.hardware.ModuleHardware.__init__(self, robot_brain, lizard_code, core_message_fields)
        DeltaArm.__init__(self, config)

    def move_to_position(self, y: float, z: float) -> bool:
        # TODO
        return False


class DeltaArmSimulation(DeltaArm, rosys.hardware.ModuleSimulation):
    def move_to_position(self, y: float, z: float) -> bool:
        if not self.is_position_reachable(y, z):
            self.log.warning('Requested (y=%.3f, z=%.3f) is outside reachable workspace; preventing overextension.', y, z)
            return False
        theta_left, theta_right = self.calculate_angles(y, z)
        if not self.check_angles(theta_left, theta_right):
            self.log.warning(
                'Angles out of bounds: left=%.2f°, right=%.2f°',
                np.rad2deg(theta_left), np.rad2deg(theta_right))
            return False
        self.end_effector_frame.y = y
        self.end_effector_frame.z = z
        self.move_to_angles(theta_left, theta_right)
        # self.log.debug('Moving to %s with %s', self.tool_position, (np.rad2deg(theta1), np.rad2deg(theta2)))
        return True

    def simulate_work_area(self) -> list[Point]:
        self.log.debug('Simulating work area')
        # Sweep a plausible YZ area
        search_distance_y = self.y_limit if self.y_limit is not None else (self.b + self.l1)
        search_distance_z = self.l1 + self.l2
        points = []
        for y in np.linspace(-search_distance_y, search_distance_y, 31):
            for z in np.linspace(-search_distance_z, 0, 31):
                success = self.move_to_position(y, z)
                if success:
                    # NOTE: Use projection convention where x corresponds to Y and y to Z
                    points.append(Point(x=y, y=z))
        return points


def projection_yz(point3d: Point3d) -> Point:
    return Point(x=point3d.y, y=point3d.z)
