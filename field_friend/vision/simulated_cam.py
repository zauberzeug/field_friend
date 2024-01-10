from typing import Optional

import rosys


class SimulatedCam(rosys.vision.SimulatedCamera, rosys.vision.CalibratableCamera):

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self.focal_length: Optional[float] = None
