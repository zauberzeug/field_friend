import rosys
from nicegui import ui


class robot(rosys.driving.robot_object):

    def __init__(self, odometer: rosys.driving.Odometer) -> None:
        super().__init__(rosys.geometry.Prism.default_robot_shape(), odometer)
        ui.add_static_files('/assets', 'assets')
        self.with_stl('assets/field_friend.stl', x=-0.15, y=-0.3, z=0.05, scale=0.001, color='#6E93D6')
