from copy import deepcopy
from typing import Optional, Self

import numpy as np
import rosys


class SimulatedCam(rosys.vision.SimulatedCamera, rosys.vision.CalibratableCamera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length: Optional[float] = None
        self.mounting: Optional[rosys.vision.calibration.Extrinsics] = None

    @classmethod
    def create_calibrated(cls, *,
                          width: int = 800, height: int = 600,
                          x: float = 0.0, y: float = 0.0, z: float = 1.0,
                          roll: float = np.pi, pitch: float = 0.0, yaw: float = 0.0,
                          **kwargs) -> Self:
        camera = cls(**kwargs)
        camera.set_perfect_calibration(width=width, height=height, x=x, y=y, z=z,
                                       roll=roll, pitch=pitch, yaw=yaw, focal_length=750)
        assert camera.calibration is not None
        camera.mounting = deepcopy(camera.calibration.extrinsics)
        return camera

    def update_calibration(self, pose: rosys.geometry.Pose) -> None:
        # TODO remove this when RoSys supports multiple extrinsics (see https://github.com/zauberzeug/rosys/discussions/130)
        assert self.mounting is not None
        assert self.calibration is not None
        new_translation = pose.transform3d(rosys.geometry.Point3d.from_tuple(self.mounting.translation))
        pose_rotation = rosys.geometry.Rotation.from_euler(0, 0, pose.yaw)
        new_rotation = pose_rotation * self.mounting.rotation
        self.calibration.extrinsics.rotation = new_rotation
        self.calibration.extrinsics.translation[:] = new_translation.tuple
