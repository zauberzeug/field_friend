
import rosys


class robot(rosys.driving.robot_object):

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
        super().__init__(rosys.geometry.Prism.default_robot_shape(), odometer)
