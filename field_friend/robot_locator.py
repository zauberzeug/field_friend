import logging
from dataclasses import dataclass
from typing import Any, Self

import numpy as np
import rosys
import rosys.helpers
from nicegui import ui
from rosys.geometry import Point, Pose, Pose3d, Rotation, Velocity
from rosys.hardware import Gnss, GnssMeasurement, Imu, ImuMeasurement, Wheels, WheelsSimulation

from .config.configuration import GnssConfiguration


@dataclass(slots=True, kw_only=True)
class State:
    timestamp: float
    x: np.ndarray
    sxx: np.ndarray

    @staticmethod
    def zero(size: int = 3) -> Self:
        return State(timestamp=rosys.time(), x=np.zeros((size, 1)), sxx=np.zeros((size, size)))

    def pose(self) -> Pose:
        return Pose(x=self.x[0, 0], y=self.x[1, 0], yaw=self.x[2, 0], time=self.timestamp)

    def point(self) -> Point:
        return Point(x=self.x[0, 0], y=self.x[1, 0])


class RobotLocator(rosys.persistence.Persistable):
    R_ODOM_LINEAR = 0.1
    R_ODOM_ANGULAR = 0.097
    R_IMU_ANGULAR = 0.01
    ODOMETRY_ANGULAR_WEIGHT = 0.1

    def __init__(self,
                 wheels: Wheels, *,
                 gnss: Gnss | None = None,
                 imu: Imu | None = None,
                 gnss_config: GnssConfiguration | None = None) -> None:
        """Robot Locator based on an extended Kalman filter."""
        super().__init__()
        self.log = logging.getLogger('field_friend.robot_locator')

        self._wheels = wheels
        self._gnss = gnss
        self._imu = imu
        self._gnss_config = gnss_config

        self.pose_frame = Pose3d().as_frame('field_friend.robot_locator')
        self._state = State.zero()
        self._first_prediction_done = False  # NOTE: the prediction step needs to be run once before the first GNSS update

        self._ignore_gnss = gnss is None
        self._ignore_imu = imu is None
        self._auto_tilt_correction = True
        self._r_odom_linear = self.R_ODOM_LINEAR
        self._r_odom_angular = self.R_ODOM_ANGULAR
        self._r_imu_angular = self.R_IMU_ANGULAR
        self._odometry_angular_weight = self.ODOMETRY_ANGULAR_WEIGHT

        self._previous_imu_measurement: ImuMeasurement | None = None
        self._wheels.VELOCITY_MEASURED.register(self._handle_velocity_measurement)
        if self._gnss is not None:
            self._gnss.NEW_MEASUREMENT.register(self._handle_gnss_measurement)
        rosys.on_startup(self.reset)

    @property
    def pose(self) -> Pose:
        return self._state.pose()

    @property
    def prediction(self) -> Pose:
        return self.pose

    async def _handle_velocity_measurement(self, velocities: list[Velocity]) -> None:
        """Implements the 'prediction' step of the Kalman filter."""
        for velocity in velocities:
            current_state = self._state
            dt = velocity.time - current_state.timestamp
            if (not self._first_prediction_done) and (self._imu is not None):
                self._previous_imu_measurement = self._imu.last_measurement
            if velocity.linear == 0 and velocity.angular == 0 and self._first_prediction_done:
                # NOTE: The robot is not moving, so we don't need to update the state
                continue
            v = velocity.linear
            omega = velocity.angular
            r_angular = self._r_odom_angular
            if not self._ignore_imu and self._imu is not None:
                imu_omega = self._get_imu_angular_velocity()
                if imu_omega is not None:
                    v, omega = self._combine_odom_imu(v, omega, imu_omega)
                    r_angular = self._odometry_angular_weight * self._r_odom_angular + \
                        (1 - self._odometry_angular_weight) * self._r_imu_angular

            theta = current_state.x[2, 0]
            theta_new = theta + omega * dt
            theta_avg = (theta + theta_new) / 2  # Average orientation
            x_new = current_state.x[0, 0] + v * np.cos(theta_avg) * dt
            y_new = current_state.x[1, 0] + v * np.sin(theta_avg) * dt
            F = np.array([
                [1, 0, -v * np.sin(theta_avg) * dt],
                [0, 1, v * np.cos(theta_avg) * dt],
                [0, 0, 1],
            ])
            R = np.array([
                [(self._r_odom_linear * dt * np.cos(theta_avg))**2, 0, 0],
                [0, (self._r_odom_linear * dt * np.sin(theta_avg))**2, 0],
                [0, 0, (r_angular * dt)**2]
            ])
            sxx_new = F @ current_state.sxx @ F.T + R
            self._state = State(timestamp=velocity.time, x=np.array(
                [[x_new, y_new, theta_new]], dtype=float).T, sxx=sxx_new)
            self._update_frame()
            self._first_prediction_done = True

    def _get_imu_angular_velocity(self) -> float | None:
        if self._previous_imu_measurement is None or self._imu is None or self._imu.last_measurement is None:
            return None
        new_imu_measurement = self._imu.last_measurement
        imu_dtheta = rosys.helpers.angle(self._previous_imu_measurement.rotation.yaw,
                                         new_imu_measurement.rotation.yaw)
        imu_dt = new_imu_measurement.time - self._previous_imu_measurement.time
        imu_angular_velocity = imu_dtheta / imu_dt if imu_dt > 0 else None
        self._previous_imu_measurement = new_imu_measurement
        return imu_angular_velocity

    def _combine_odom_imu(self, odometry_v: float, odometry_omega: float, imu_omega: float) -> tuple[float, float]:
        """Linear combination of odometry and IMU angular velocity.
        The linear velocity is adjusted based on the angular velocity difference (indicating a slip)."""
        v = odometry_v
        ow = self._odometry_angular_weight
        omega = ow * odometry_omega + (1 - ow) * imu_omega
        # Adjust linear velocity based on angular velocity difference
        angular_difference = abs(odometry_omega - imu_omega)
        correction_factor = 1.0 / (1.0 + angular_difference)
        v = ow * v + (1.0 - ow) * (v * correction_factor)
        return v, omega

    def _handle_gnss_measurement(self, gnss_measurement: GnssMeasurement) -> None:
        """Triggers the 'update' step of the Kalman filter."""
        if self._ignore_gnss:
            return
        if not np.isfinite(gnss_measurement.heading_std_dev):
            # normally we would only handle the position if no heading is available,
            # but the field friend needs the rtk accuracy to function properly
            return
        pose, r_xy, r_theta = self._get_local_pose_and_uncertainty(gnss_measurement)
        if self._auto_tilt_correction and isinstance(self._imu, Imu) and not self._ignore_imu and self._imu.last_measurement is not None:
            pose = self._correct_gnss_with_imu(pose)
        z = [[pose.x], [pose.y], [pose.yaw]]
        current_state = self._state
        h = [[current_state.x[0, 0]], [current_state.x[1, 0]], [current_state.x[2, 0]]]
        H = np.eye(3)
        variance = np.array([r_xy, r_xy, r_theta], dtype=np.float64)**2
        Q = np.diag(variance)
        self._update(z=np.array(z), h=np.array(h), H=H, Q=Q)

    def _get_local_pose_and_uncertainty(self, gnss_measurement: GnssMeasurement) -> tuple[Pose, float, float]:
        current_pose = self._state.pose()
        pose = gnss_measurement.pose.to_local()
        pose.yaw = current_pose.yaw + rosys.helpers.angle(current_pose.yaw, pose.yaw)
        r_xy = (gnss_measurement.latitude_std_dev + gnss_measurement.longitude_std_dev) / 2
        r_theta = np.deg2rad(gnss_measurement.heading_std_dev)
        return pose, r_xy, r_theta

    def _correct_gnss_with_imu(self, pose: Pose) -> Pose:
        assert isinstance(self._imu, Imu)
        assert self._imu.last_measurement is not None
        assert self._gnss_config is not None
        roll = self._imu.last_measurement.rotation.roll
        pitch = self._imu.last_measurement.rotation.pitch
        antenna_roll_correction = Pose(x=0, y=self._gnss_config.y * (1 - np.cos(roll)), yaw=0)
        height_correction = Pose(x=self._gnss_config.z * np.sin(-pitch),
                                 y=self._gnss_config.z * np.sin(roll), yaw=0)
        return pose.transform_pose(antenna_roll_correction).transform_pose(height_correction)

    def _update(self, *, z: np.ndarray, h: np.ndarray, H: np.ndarray, Q: np.ndarray) -> None:  # noqa: N803
        current_state = self._state
        S = H @ current_state.sxx @ H.T + Q
        try:
            L = np.linalg.cholesky(S)  # Use Cholesky decomposition for numerical stability
            K = current_state.sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        except np.linalg.LinAlgError:
            S += np.eye(S.shape[0]) * 1e-6
            L = np.linalg.cholesky(S)
            K = current_state.sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        x_new = current_state.x + K @ (z - h)
        sxx_new = (np.eye(current_state.sxx.shape[0]) - K @ H) @ current_state.sxx
        self._state = State(timestamp=rosys.time(), x=x_new, sxx=sxx_new)
        self._update_frame()

    def _update_frame(self) -> None:
        pose = self._state.pose()
        self.pose_frame.x = pose.x
        self.pose_frame.y = pose.y
        self.pose_frame.rotation = Rotation.from_euler(0, 0, pose.yaw)

    async def reset(self, *, gnss_timeout: float = 2.0) -> None:
        reset_pose = Pose(x=0.0, y=0.0, yaw=0.0)
        r_xy = 0.0
        r_theta = 0.0
        if isinstance(self._wheels, WheelsSimulation):
            self._wheels.pose = Pose(time=rosys.time())
        if self._gnss is not None and not self._ignore_gnss:
            try:
                await self._gnss.NEW_MEASUREMENT.emitted(gnss_timeout)
                assert self._gnss.last_measurement is not None
                reset_pose, r_xy, r_theta = self._get_local_pose_and_uncertainty(self._gnss.last_measurement)
            except TimeoutError:
                self.log.error('GNSS timeout while resetting position. Activate _ignore_gnss to use zero position.')
                return
            except AssertionError:
                self.log.error(
                    'GNSS measurement is not available while resetting position. Activate _ignore_gnss to use zero position.')
                return
        x = np.array([[reset_pose.x, reset_pose.y, reset_pose.yaw]], dtype=float).T
        variance = np.array([r_xy, r_xy, r_theta], dtype=np.float64)**2
        sxx = np.diag(variance)
        self._state = State(timestamp=rosys.time(), x=x, sxx=sxx)
        self._update_frame()
        rosys.notify('Positioning initialized', 'positive')

    def backup_to_dict(self) -> dict[str, Any]:
        return {
            'r_odom_linear': self._r_odom_linear,
            'r_odom_angular': self._r_odom_angular,
            'r_imu_angular': self._r_imu_angular,
            'odometry_angular_weight': self._odometry_angular_weight,
        }

    def restore_from_dict(self, data: dict[str, Any]) -> None:
        self._r_odom_linear = data.get('r_odom_linear', self.R_ODOM_LINEAR)
        self._r_odom_angular = data.get('r_odom_angular', self.R_ODOM_ANGULAR)
        self._r_imu_angular = data.get('r_imu_angular', self.R_IMU_ANGULAR)
        self._odometry_angular_weight = data.get('odometry_angular_weight', self.ODOMETRY_ANGULAR_WEIGHT)

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Kalman Filter').classes('text-center text-bold')
            with ui.grid(columns=2).classes('w-full gap-0'):
                ui.label().bind_text_from(self, 'pose', lambda p: f'x: {p.x:.3f}m')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {m[0, 0]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'y: {p.y:.3f}m')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {m[1, 1]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'θ: {p.yaw_deg:.2f}°')
                ui.label().bind_text_from(self, '_Sxx', lambda m: f'± {np.rad2deg(m[2, 2]):.2f}°')

            with ui.grid(columns=2).classes('w-full'):
                ui.checkbox('Ignore GNSS', value=self._ignore_gnss).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, '_ignore_gnss').tooltip('Ignore GNSS measurements. When deactivated, reset the filter for better positioning.')
                ui.checkbox('Ignore IMU', value=self._ignore_imu).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, '_ignore_imu')
                ui.checkbox('Correct GNSS with IMU', value=self._auto_tilt_correction).props('dense').classes('col-span-2') \
                    .bind_value_to(self, '_auto_tilt_correction')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R v linear', min=0, step=0.01, format='%.3f', suffix='m/s', value=self._r_odom_linear, on_change=self.request_backup) \
                        .bind_value_to(self, '_r_odom_linear')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω odom', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self._r_odom_angular), on_change=self.request_backup) \
                        .bind_value_to(self, '_r_odom_angular', forward=np.deg2rad)
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω imu', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self._r_imu_angular), on_change=self.request_backup) \
                        .bind_value_to(self, '_r_imu_angular', forward=np.deg2rad)
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='ω odom weight', min=0, step=0.01, format='%.3f', value=self._odometry_angular_weight, on_change=self.request_backup) \
                        .bind_value_to(self, '_odometry_angular_weight')

            ui.button('Reset', on_click=self.reset) \
                .tooltip('Reset the position to the GNSS measurement or zero position if GNSS is not available or ignored.')
