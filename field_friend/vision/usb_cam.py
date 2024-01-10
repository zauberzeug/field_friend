from typing import Optional

import rosys


class UsbCam(rosys.vision.UsbCamera, rosys.vision.CalibratableCamera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length: Optional[float] = None
