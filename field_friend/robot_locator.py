import logging
from typing import Any

import numpy as np
import rosys
import rosys.helpers
from nicegui import ui
from rosys.geometry import Pose3d, Rotation
from rosys.hardware import Gnss, GnssMeasurement, Imu, ImuMeasurement, Wheels


class RobotLocator(rosys.persistence.PersistentModule):

    def __init__(self, wheels: Wheels, gnss: Gnss, imu: Imu) -> None:
        """ Robot Locator based on an extended Kalman filter.

        ### State

        ```py
        x = [
            x,       # x position
            y,       # y position
            theta,   # orientation
            v,       # linear velocity
            omega,   # angular velocity
            a        # linear acceleration
        ]
        ```

        ### Process Model

        ```py
        x = x + v * cos(theta) * dt
        y = y + v * sin(theta) * dt
        theta = theta + omega * dt
        v = v + a * dt
        omega = omega
        a = a

        F = [
            [1, 0, -v * sin(theta) * dt, cos(theta) * dt, 0, 0],
            [0, 1, v * cos(theta) * dt, sin(theta) * dt, 0, 0],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ]
        ```

        ### Odometry Measurement Model

        ```py
        z = [
            v,
            omega,
        ]

        H = [
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
        ]
        ```

        ### GNSS Measurement Model

        ```py
        z = [
            x,
            y,
            theta,
        ]

        H = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ]
        ```
        """
        super().__init__(persistence_key='field_friend.robot_locator')
        self.log = logging.getLogger('field_friend.robot_locator')

        self.wheels = wheels
        self.gnss = gnss
        self.imu = imu

        self.pose_frame = Pose3d().as_frame('field_friend.robot_locator')
        self.x = np.zeros((6, 1))
        self.Sxx = np.zeros((6, 6))
        self.t = rosys.time()

        self.ignore_odometry = False
        self.ignore_gnss = False
        self.ignore_imu = False
        self.q_odometry_v = 0.01
        self.q_odometry_omega = 0.01
        self.q_imu_yaw = 0.01
        self.r_x = 0.01
        self.r_y = 0.01
        self.r_theta = 0.01
        self.r_v = 0.01
        self.r_omega = 1.0
        self.r_a = 1.0

        self.wheels.VELOCITY_MEASURED.register(self.handle_velocity_measurement)
        self.gnss.NEW_MEASUREMENT.register(self.handle_gnss_measurement)
        self.imu.NEW_MEASUREMENT.register(self.handle_imu_measurement)

    def backup(self) -> dict[str, Any]:
        return {
            'q_odometry_v': self.q_odometry_v,
            'q_odometry_omega': self.q_odometry_omega,
            'q_imu_yaw': self.q_imu_yaw,
            'r_x': self.r_x,
            'r_y': self.r_y,
            'r_theta': self.r_theta,
            'r_v': self.r_v,
            'r_omega': self.r_omega,
            'r_a': self.r_a,
        }

    def restore(self, data: dict[str, Any]) -> None:
        self.q_odometry_v = data.get('q_odometry_v', 0.01)
        self.q_odometry_omega = data.get('q_odometry_omega', 0.01)
        self.q_imu_yaw = data.get('q_imu_yaw', 0.01)
        self.r_x = data.get('r_x', 0.01)
        self.r_y = data.get('r_y', 0.01)
        self.r_theta = data.get('r_theta', 0.01)
        self.r_v = data.get('r_v', 0.01)
        self.r_omega = data.get('r_omega', 1.00)
        self.r_a = data.get('r_a', 1.00)

    @property
    def pose(self) -> rosys.geometry.Pose:
        return rosys.geometry.Pose(
            x=self.x[0, 0],
            y=self.x[1, 0],
            yaw=self.x[2, 0],
        )

    @property
    def prediction(self) -> rosys.geometry.Pose:
        return self.pose

    @property
    def velocity(self) -> rosys.geometry.Velocity:
        return rosys.geometry.Velocity(
            linear=self.x[3, 0],
            angular=self.x[4, 0],
            time=self.t,
        )

    def handle_velocity_measurement(self, velocities: list[rosys.geometry.Velocity]) -> None:
        if self.ignore_odometry:
            return
        for velocity in velocities:
            self.predict(velocity.time)
            self.update(
                z=np.array([[velocity.linear, velocity.angular]]).T,
                h=np.array([[self.x[3, 0], self.x[4, 0]]]).T,
                H=np.array([
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                ]),
                Q=np.diag([self.q_odometry_v, self.q_odometry_omega])**2,
            )

    def handle_gnss_measurement(self, gnss_measurement: GnssMeasurement) -> None:
        if self.ignore_gnss:
            return
        if not np.isfinite(gnss_measurement.heading_std_dev):
            # normally we would only handle the position if no heading is available,
            # but the field friend needs the rtk accuracy to function properly
            return
        self.predict(gnss_measurement.time)
        pose = gnss_measurement.pose.to_local()
        pose.yaw = self.x[2, 0] + rosys.helpers.angle(self.x[2, 0], pose.yaw)
        z = [[pose.x], [pose.y], [pose.yaw]]
        h = [[self.x[0, 0]], [self.x[1, 0]], [self.x[2, 0]]]
        H = [
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        ]
        r_xy = (gnss_measurement.latitude_std_dev + gnss_measurement.longitude_std_dev) / 2
        r_theta = np.deg2rad(gnss_measurement.heading_std_dev)
        Q = np.diag([r_xy, r_xy, r_theta])**2
        self.update(z=np.array(z), h=np.array(h), H=np.array(H), Q=Q)

    def handle_imu_measurement(self, imu_measurement: ImuMeasurement) -> None:
        if self.ignore_imu:
            return
        self.predict(imu_measurement.time)
        z = [[imu_measurement.yaw_velocity]]
        h = [[self.x[4, 0]]]
        H = [
            [0, 0, 0, 0, 1, 0]
        ]
        Q = np.diag([self.q_imu_yaw])**2
        self.update(z=np.array(z), h=np.array(h), H=np.array(H), Q=Q)

    def predict(self, time: float | None = None) -> None:
        if time is None:
            time = rosys.time()
        dt = time - self.t
        self.t = time

        theta = self.x[2, 0]
        v = self.x[3, 0]
        omega = self.x[4, 0]
        a = self.x[5, 0]

        F = np.array([
            [1, 0, -v * np.sin(theta) * dt, np.cos(theta) * dt, 0, 0],
            [0, 1, v * np.cos(theta) * dt, np.sin(theta) * dt, 0, 0],
            [0, 0, 1, 0, dt, 0],
            [0, 0, 0, 1, 0, dt],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1],
        ])

        R = (np.diag([self.r_x, self.r_y, self.r_theta, self.r_v, self.r_omega, self.r_a]) * dt)**2

        self.x[0] += v * np.cos(theta) * dt
        self.x[1] += v * np.sin(theta) * dt
        self.x[2] += omega * dt
        self.x[3] += a * dt
        self.Sxx = F @ self.Sxx @ F.T + R
        self.update_frame()

    def update(self, *, z: np.ndarray, h: np.ndarray, H: np.ndarray, Q: np.ndarray) -> None:  # noqa: N803
        K = self.Sxx @ H.T @ np.linalg.inv(H @ self.Sxx @ H.T + Q)
        self.x = self.x + K @ (z - h)
        self.Sxx = (np.eye(self.Sxx.shape[0]) - K @ H) @ self.Sxx
        self.update_frame()

    def update_frame(self) -> None:
        self.pose_frame.x = self.x[0, 0]
        self.pose_frame.y = self.x[1, 0]
        self.pose_frame.rotation = Rotation.from_euler(0, 0, self.x[2, 0])

    def reset(self, *, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x = np.array([[x, y, yaw, 0, 0, 0]], dtype=float).T
        self.Sxx = np.diag([self.r_x, self.r_y, self.r_theta, self.r_v, self.r_omega, self.r_a])**2.0

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
                ui.label().bind_text_from(self, 'velocity', lambda v: f'v: {v.linear:.2f}m/s')
                ui.label().bind_text_from(self, 'Sxx', lambda m: f'± {m[3, 3]:.2f}m/s')
                ui.label().bind_text_from(self, 'velocity', lambda v: f'ω: {np.rad2deg(v.angular): 2.2f}°/s')
                ui.label().bind_text_from(self, 'Sxx', lambda m: f'± {np.rad2deg(m[4, 4]):.2f}°/s')
            with ui.grid(columns=2).classes('w-full'):
                ui.checkbox('Ignore Odometry').props('dense color=red').classes('col-span-2') \
                    .bind_value(self, 'ignore_odometry')
                ui.checkbox('Ignore GNSS').props('dense color=red').classes('col-span-2') \
                    .bind_value(self, 'ignore_gnss')
                ui.checkbox('Ignore IMU').props('dense color=red').classes('col-span-2') \
                    .bind_value(self, 'ignore_imu')
                for key, label in {
                    'q_odometry_v': 'Q Odometry v',
                    'q_odometry_omega': 'Q Odometry ω',
                    'q_imu_yaw': 'Q IMU yaw',
                    'r_x': 'R x',
                    'r_y': 'R y',
                    'r_theta': 'R θ',
                    'r_v': 'R v',
                    'r_omega': 'R ω',
                    'r_a': 'R a',
                }.items():
                    with ui.column().classes('w-full gap-0'):
                        ui.number(label, min=0, step=0.01, format='%.6f', on_change=self.request_backup) \
                            .bind_value(self, key)
            ui.button('Reset', on_click=self.reset)
