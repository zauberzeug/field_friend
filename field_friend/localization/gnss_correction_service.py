import numpy as np
import rosys

from .gnss import Gnss


class GnssCorrectionService:
    def __init__(self, imu: rosys.hardware.Imu | None, gnss: Gnss, robot_height: float) -> None:
        self.imu = imu
        self.gnss = gnss
        self.CORRECTED_ROBOT_POSE = rosys.event.Event()
        """the robot pose corrected for the robot roll (argument: Pose)"""
        if self.imu:
            self.imu.NEW_MEASUREMENT.register(self.update_imu)
        self.gnss.ROBOT_POSE_LOCATED.register(self.update_gnss)
        self._last_offset = 0.0
        self._robot_height = robot_height

    def update_imu(self, euler: tuple[float, float, float]) -> None:
        roll = euler[0]
        pitch = euler[1]
        self._last_offset_y = self._robot_height * np.sin(np.radians(roll))
        self._last_offset_x = self._robot_height * np.sin(np.radians(pitch))

    def update_gnss(self, measured_pose: rosys.geometry.Pose) -> None:
        corrected_pose = measured_pose.transform_pose(rosys.geometry.Pose(
            x=self._last_offset_x,
            y=self._last_offset_y,
            yaw=0,
            time=measured_pose.time,
        ))
        self.CORRECTED_ROBOT_POSE.emit(corrected_pose)
