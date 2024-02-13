from typing import Any, Optional, Self

import rosys
from rosys import persistence


class UsbCam(rosys.vision.CalibratableCamera, rosys.vision.UsbCamera):

    def __init__(self,
                 *,
                 calibration: Optional[rosys.vision.Calibration] = None,
                 focal_length: Optional[float] = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length = focal_length
        self.calibration = calibration

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        return cls(**data | {
            'calibration': persistence.from_dict(rosys.vision.Calibration, data['calibration']) if data.get('calibration') else None,
        })

    def to_dict(self) -> dict[str, Any]:
        return super().to_dict() | {
            'focal_length': self.focal_length,
            'calibration': persistence.to_dict(self.calibration),
        }
