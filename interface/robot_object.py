import rosys
from nicegui import ui
from nicegui.elements.scene_objects import Box, Cylinder, Group

import hardware


class robot_object(rosys.driving.robot_object):
    width = 0.63
    length = 0.78
    offset = 0.16
    chain_width = 0.145

    shape = rosys.geometry.Prism(
        outline=[
            (-offset, -width/2),
            (length - offset, -width/2),
            (length - offset, -width/2 + chain_width),
            (0, -width/2 + chain_width),
            (0, width/2 - chain_width),
            (length - offset, width/2 - chain_width),
            (length - offset, width/2),
            (-offset, width/2)
        ],
        height=0.40,
    )

    def __init__(self, odometer: rosys.driving.Odometer, camera_provider: rosys.vision.CameraProvider,
                 robot: hardware.Robot) -> None:
        super().__init__(self.shape, odometer, debug=True)

        self.robot = robot
        self.odometer = odometer

        ui.add_static_files('/assets', 'assets')
        self.with_stl('assets/field_friend.stl', x=-0.15, y=-0.3, z=0.05, scale=0.001, color='#6E93D6')
        with self:
            with Group() as self.camera:
                rosys.vision.camera_objects(camera_provider, rosys.vision.CameraProjector(camera_provider), debug=True)
            with Group() as self.tool:
                Box(width=0.05, height=0.63, depth=0.08).move(
                    x=self.robot.AXIS_OFFSET_X+0.025, z=0.34).material('#6E93D6', 1.0)
                Box(width=0.02, height=0.02, depth=0.3).move(
                    x=self.robot.AXIS_OFFSET_X, z=0.4).material('#3A3E42', 1.0)

    def update(self) -> None:
        super().update()
        self.tool.move(y=self.robot.yaxis_position, z=self.robot.zaxis_position)
