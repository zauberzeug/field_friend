from copy import deepcopy
from typing import Optional, Self

import numpy as np
import rosys


class SimulatedCam(rosys.vision.SimulatedCamera, rosys.vision.CalibratableCamera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length: Optional[float] = None
        self.mount_position: Optional[rosys.geometry.Point3d] = None
        self.mounting: Optional[rosys.vision.calibration.Extrinsics] = None

    @classmethod
    def create_calibrated(cls, *,
                          width: int = 800, height: int = 600,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
                          **kwargs) -> Self:
        camera = cls(**kwargs)
        camera.set_perfect_calibration(width=width, height=height, x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)
        assert camera.calibration is not None
        camera.mounting = deepcopy(camera.calibration.extrinsics)
        return camera

    def update_calibration(self, pose: rosys.geometry.Pose) -> None:
        assert self.mounting is not None
        assert self.calibration is not None
        new_translation = pose.transform3d(rosys.geometry.Point3d.from_list(self.mounting.translation))
        pose_rotation = rosys.geometry.Rotation.from_euler(0, 0, pose.yaw)  # Assuming pose contains only yaw for 2D
        new_rotation = pose_rotation * self.mounting.rotation
        self.calibration = rosys.vision.calibration.Calibration(
            intrinsics=self.calibration.intrinsics,
            extrinsics=rosys.vision.calibration.Extrinsics(rotation=new_rotation, translation=[
                new_translation.x, new_translation.y, new_translation.z])
        )
