import logging
from typing import Any

import numpy as np
import rosys
import rosys.helpers
from nicegui import ui
from rosys.geometry import Pose3d, Rotation
from rosys.hardware import Gnss, GnssMeasurement, Imu, ImuMeasurement, Wheels


class RobotLocator(rosys.persistence.PersistentModule):
    R_ODOM_LINEAR = 0.1
    R_ODOM_ANGULAR = 0.097
    R_IMU_ANGULAR = 0.01
    ODOMETRY_ANGULAR_WEIGHT = 0.1

    def __init__(self, wheels: Wheels, gnss: Gnss, imu: Imu | None = None) -> None:
        """ Robot Locator based on an extended Kalman filter."""
        super().__init__(persistence_key='field_friend.robot_locator')
        self.log = logging.getLogger('field_friend.robot_locator')

        self.wheels = wheels
        self.gnss = gnss
        self.imu = imu

        self.pose_frame = Pose3d().as_frame('field_friend.robot_locator')

        state_size = 3
        self.x = np.zeros((state_size, 1))
        self.Sxx = np.zeros((state_size, state_size))
        self.t = rosys.time()

        # bound attributes
        self.ignore_gnss = False
        self.ignore_imu = False
        self.r_odom_linear = self.R_ODOM_LINEAR
        self.r_odom_angular = self.R_ODOM_ANGULAR
        self.r_imu_angular = self.R_IMU_ANGULAR
        self.odometry_angular_weight = self.ODOMETRY_ANGULAR_WEIGHT

        self._first_prediction_done = False
        self._previous_imu_measurement: ImuMeasurement | None = None

        self.wheels.VELOCITY_MEASURED.register(self._handle_velocity_measurement)
        self.gnss.NEW_MEASUREMENT.register(self._handle_gnss_measurement)

    def backup(self) -> dict[str, Any]:
        return {
            'r_odom_linear': self.r_odom_linear,
            'r_odom_angular': self.r_odom_angular,
            'r_imu_angular': self.r_imu_angular,
            'odometry_angular_weight': self.odometry_angular_weight,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.r_odom_linear = data.get('r_odom_linear', self.R_ODOM_LINEAR)
        self.r_odom_angular = data.get('r_odom_angular', self.R_ODOM_ANGULAR)
        self.r_imu_angular = data.get('r_imu_angular', self.R_IMU_ANGULAR)
        self.odometry_angular_weight = data.get('odometry_angular_weight', self.ODOMETRY_ANGULAR_WEIGHT)

    @property
    def pose(self) -> rosys.geometry.Pose:
        return rosys.geometry.Pose(
            x=self.x[0, 0],
            y=self.x[1, 0],
            yaw=self.x[2, 0],
            time=self.t,
        )

    @property
    def prediction(self) -> rosys.geometry.Pose:
        return self.pose

    async def _handle_velocity_measurement(self, velocities: list[rosys.geometry.Velocity]) -> None:
        """Implements the 'prediction' step of the Kalman filter."""
        for velocity in velocities:
            dt = velocity.time - self.t
            self.t = velocity.time
            if (not self._first_prediction_done) and (self.imu is not None):
                self._last_imu_measurement = self.imu.last_measurement

            if velocity.linear == 0 and velocity.angular == 0 and self._first_prediction_done:
                # The robot is not moving, so we don't need to update the state
                continue

            v = velocity.linear
            omega = velocity.angular
            r_angular = self.r_odom_angular
            if not self.ignore_imu:
                imu_omega = self._get_imu_angular_velocity()
                if imu_omega is not None:
                    v, omega = self._combine_odom_imu(v, omega, imu_omega)
                    r_angular = self.odometry_angular_weight * self.r_odom_angular + \
                        (1 - self.odometry_angular_weight) * self.r_imu_angular

            theta = self.x[2, 0]
            theta_new = theta + omega * dt
            theta_avg = (theta + theta_new) / 2  # Average orientation

            self.x[0, 0] += v * np.cos(theta_avg) * dt
            self.x[1, 0] += v * np.sin(theta_avg) * dt
            self.x[2, 0] = theta_new

            F = np.array([
                [1, 0, -v * np.sin(theta_avg) * dt],
                [0, 1, v * np.cos(theta_avg) * dt],
                [0, 0, 1],
            ])

            R = np.array([
                [(self.r_odom_linear * dt * np.cos(theta_avg))**2, 0, 0],
                [0, (self.r_odom_linear * dt * np.sin(theta_avg))**2, 0],
                [0, 0, (r_angular * dt)**2]
            ])
            self.Sxx = F @ self.Sxx @ F.T + R
            self._update_frame()
            self._first_prediction_done = True

    def _get_imu_angular_velocity(self) -> float | None:
        if self._previous_imu_measurement is None or self.imu is None or self.imu.last_measurement is None:
            return None
        new_imu_measurement = self.imu.last_measurement
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
        ow = self.odometry_angular_weight
        omega = ow * odometry_omega + (1 - ow) * imu_omega
        # Adjust linear velocity based on angular velocity difference
        angular_difference = abs(odometry_omega - imu_omega)
        correction_factor = 1.0 / (1.0 + angular_difference)
        v = ow * v + (1.0 - ow) * (v * correction_factor)
        return v, omega

    def _handle_gnss_measurement(self, gnss_measurement: GnssMeasurement) -> None:
        """Triggers the 'update' step of the Kalman filter."""
        if not self._first_prediction_done:
            return
        if self.ignore_gnss:
            return
        if not np.isfinite(gnss_measurement.heading_std_dev):
            # normally we would only handle the position if no heading is available,
            # but the field friend needs the rtk accuracy to function properly
            return
        pose = gnss_measurement.pose.to_local()
        pose.yaw = self.x[2, 0] + rosys.helpers.angle(self.x[2, 0], pose.yaw)
        z = [[pose.x], [pose.y], [pose.yaw]]
        h = [[self.x[0, 0]], [self.x[1, 0]], [self.x[2, 0]]]
        H = np.eye(3)
        r_xy = (gnss_measurement.latitude_std_dev + gnss_measurement.longitude_std_dev) / 2
        r_theta = np.deg2rad(gnss_measurement.heading_std_dev)
        Q = np.diag([r_xy, r_xy, r_theta])**2
        self._update(z=np.array(z), h=np.array(h), H=H, Q=Q)

    def _update(self, *, z: np.ndarray, h: np.ndarray, H: np.ndarray, Q: np.ndarray) -> None:  # noqa: N803
        S = H @ self.Sxx @ H.T + Q
        # Use Cholesky decomposition for numerical stability
        try:
            L = np.linalg.cholesky(S)
            K = self.Sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        except np.linalg.LinAlgError:
            S += np.eye(S.shape[0]) * 1e-6
            L = np.linalg.cholesky(S)
            K = self.Sxx @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(S.shape[0])))
        self.x = self.x + K @ (z - h)
        self.Sxx = (np.eye(self.Sxx.shape[0]) - K @ H) @ self.Sxx
        self._update_frame()

    def _update_frame(self) -> None:
        self.pose_frame.x = self.x[0, 0]
        self.pose_frame.y = self.x[1, 0]
        self.pose_frame.rotation = Rotation.from_euler(0, 0, self.x[2, 0])

    def _reset(self, *, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x = np.array([[x, y, yaw]], dtype=float).T
        self.Sxx = np.diag([0.0, 0.0, 0.0])

    def developer_ui(self) -> None:
        with ui.column():
            ui.label('Kalman Filter').classes('text-center text-bold')
            with ui.grid(columns=2).classes('w-full gap-0'):
                ui.label().bind_text_from(self, 'pose', lambda p: f'x: {p.x:.3f}m')
                ui.label().bind_text_from(self, 'Sxx', lambda m: f'± {m[0, 0]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'y: {p.y:.3f}m')
                ui.label().bind_text_from(self, 'Sxx', lambda m: f'± {m[1, 1]:.3f}m')
                ui.label().bind_text_from(self, 'pose', lambda p: f'θ: {p.yaw_deg:.2f}°')
                ui.label().bind_text_from(self, 'Sxx', lambda m: f'± {np.rad2deg(m[2, 2]):.2f}°')
            with ui.grid(columns=2).classes('w-full'):
                ui.checkbox('Ignore GNSS', value=self.ignore_gnss).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, 'ignore_gnss')
                ui.checkbox('Ignore IMU', value=self.ignore_imu).props('dense color=red').classes('col-span-2') \
                    .bind_value_to(self, 'ignore_imu')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R v linear', min=0, step=0.01, format='%.3f', suffix='m/s', value=self.r_odom_linear, on_change=self.request_backup) \
                        .bind_value_to(self, 'r_odom_linear')
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω odom', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self.r_odom_angular), on_change=self.request_backup) \
                        .bind_value_to(self, 'r_odom_angular', forward=np.deg2rad)
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='R ω imu', min=0, step=0.01, format='%.3f', suffix='°/s', value=np.rad2deg(self.r_imu_angular), on_change=self.request_backup) \
                        .bind_value_to(self, 'r_imu_angular', forward=np.deg2rad)
                with ui.column().classes('w-24 gap-0'):
                    ui.number(label='ω odom weight', min=0, step=0.01, format='%.3f', value=self.odometry_angular_weight, on_change=self.request_backup) \
                        .bind_value_to(self, 'odometry_angular_weight')

            ui.button('Reset', on_click=self._reset)
