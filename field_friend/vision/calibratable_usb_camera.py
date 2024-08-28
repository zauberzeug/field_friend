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
        base_dict = {
            'id': self.id,
            'name': self.name,
            'connect_after_init': self.connect_after_init,
            'streaming': self.streaming,
            'focal_length': self.focal_length,
            'calibration': persistence.to_dict(self.calibration),
        }
        return base_dict | {name: param.value for name, param in self._parameters.items()}
