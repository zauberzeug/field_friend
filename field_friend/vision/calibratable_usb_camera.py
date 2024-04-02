from typing import Any, Optional, Self

import rosys
from icecream import ic
from rosys import persistence
from rosys.vision.image import Image, ImageSize
from rosys.vision.image_processing import process_jpeg_image, process_ndarray_image, to_bytes
from rosys.vision.image_rotation import ImageRotation


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

    async def capture_image(self) -> None:
        if not self.is_connected:
            return None

        assert self.device is not None
        capture_success, captured_image = self.device.capture.read()  # FIXME: Tmp fix for camera bug with iobound call
        image_is_MJPG = 'MJPG' in self.device.video_formats
        if not capture_success:
            await self.disconnect()
            return

        if captured_image is None:
            return

        if image_is_MJPG:
            bytes_ = await rosys.run.io_bound(to_bytes, captured_image)
            if self.crop or self.rotation != ImageRotation.NONE:
                bytes_ = await rosys.run.cpu_bound(process_jpeg_image, bytes_, self.rotation, self.crop)
        else:
            bytes_ = await rosys.run.cpu_bound(process_ndarray_image, captured_image, self.rotation, self.crop)

        image_size = ImageSize(width=captured_image.shape[1], height=captured_image.shape[0])
        final_image_resolution = self._resolution_after_transform(image_size)

        image = Image(time=rosys.time(), camera_id=self.id, size=final_image_resolution, data=bytes_)
        self._add_image(image)
