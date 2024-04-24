from typing import Any, Optional, Self

import rosys
from rosys import persistence


class CalibratableUsbCamera(rosys.vision.CalibratableCamera, rosys.vision.UsbCamera):

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
        return cls(**(data | {
            'calibration': persistence.from_dict(rosys.vision.Calibration, data['calibration']) if data.get('calibration') else None,
        }))

    def to_dict(self) -> dict:
        base_dict = {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
            'focal_length': self.focal_length,
            'calibration': persistence.to_dict(self.calibration),
        }
        return base_dict | {name: param.value for name, param in self._parameters.items()}
