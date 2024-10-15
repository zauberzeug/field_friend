from typing import Any

import numpy as np
import rosys


class RobotLocator(rosys.persistence.PersistentModule):
    def __init__(self, wheels: rosys.hardware.Wheels):
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
        super().__init__(persistence_key="field_friend.robot_locator")
        self.x = np.zeros((6, 1))
        self.Sxx = np.zeros((6, 6))
        self.t = rosys.time()

        self.ignore_odometry = False

        self.q_odometry_v = 0.01
        self.q_odometry_omega = 0.01

        self.r_x = 0.01
        self.r_y = 0.01
        self.r_theta = 0.01
        self.r_v = 0.01
        self.r_omega = 1.00
        self.r_a = 1.00

        wheels.VELOCITY_MEASURED.register(self.handle_velocity_measurement)

    def backup(self) -> dict[str, Any]:
        return {
            'q_odometry_v': self.q_odometry_v,
            'q_odometry_omega': self.q_odometry_omega,
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

    def handle_velocity_measurement(self, velocities: list[rosys.geometry.Velocity]) -> None:
        self.predict()
        if not self.ignore_odometry:
            for velocity in velocities:
                self.update(
                    z=np.array([[velocity.linear, velocity.angular]]).T,
                    h=np.array([[self.x[3, 0], self.x[4, 0]]]).T,
                    H=np.array([
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                    ]),
                    Q=np.diag([self.q_odometry_v, self.q_odometry_omega])**2,
                )

    def predict(self) -> None:
        dt = rosys.time() - self.t
        self.t = rosys.time()

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

    def update(self, *, z: np.ndarray, h: np.ndarray, H: np.ndarray, Q: np.ndarray) -> None:
        K = self.Sxx @ H.T @ np.linalg.inv(H @ self.Sxx @ H.T + Q)
        self.x = self.x + K @ (z - h)
        self.Sxx = (np.eye(self.Sxx.shape[0]) - K @ H) @ self.Sxx

    def reset(self, *, x: float = 0.0, y: float = 0.0, yaw: float = 0.0) -> None:
        self.x = np.array([[x, y, yaw, 0, 0, 0]], dtype=float).T
        self.Sxx = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01])**2
