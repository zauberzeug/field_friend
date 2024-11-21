from typing import Any, Self

import rosys
from rosys import persistence


class CalibratableUsbCamera(rosys.vision.CalibratableCamera, rosys.vision.UsbCamera):

    def __init__(self,
                 *,
                 calibration: rosys.vision.Calibration | None = None,
                 focal_length: float | None = None,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length = focal_length
        self.calibration = calibration

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Self:
        if 'translation' in (data.get('calibration') or {}).get('extrinsics', {}):
            data['calibration']['extrinsics']['x'] = data['calibration']['extrinsics']['translation'][0]
            data['calibration']['extrinsics']['y'] = data['calibration']['extrinsics']['translation'][1]
            data['calibration']['extrinsics']['z'] = data['calibration']['extrinsics']['translation'][2]
            data['calibration']['extrinsics'].pop('translation')
        if 'rotation' in (data.get('calibration') or {}).get('intrinsics', {}):
            data['calibration']['extrinsics']['rotation']['R'] = (
                rosys.geometry.Rotation(R=data['calibration']['extrinsics']['rotation']['R']) *
                rosys.geometry.Rotation(R=data['calibration']['intrinsics']['rotation']['R'])
            ).R
        print(f'data: {data}')
        return cls(**(data | {
            'calibration': persistence.from_dict(rosys.vision.Calibration, data['calibration']) if data.get('calibration') else None,
        }))

    def to_dict(self) -> dict:
        return super().to_dict() | {'focal_length': self.focal_length} | {name: param.value for name, param in self._parameters.items()}
