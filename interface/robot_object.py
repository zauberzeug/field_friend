import rosys
from nicegui import app, ui
from nicegui.elements.scene_objects import Box, Group

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

        app.add_static_files('/assets', 'assets')
        self.with_stl('assets/field_friend.stl', x=-0.15, y=-0.3, z=0.05, scale=0.001, color='#6E93D6')
        with self:
            with Group() as self.camera:
                rosys.vision.camera_objects(camera_provider, rosys.vision.CameraProjector(camera_provider))
            with Group() as self.axis:
                Box(width=0.05, height=0.63, depth=0.08).move(
                    x=self.robot.AXIS_OFFSET_X+0.025, z=0.34).material('#6E93D6', 1.0)
            with Group() as self.tool:
                Box(width=0.02, height=0.02, depth=0.3).move(
                    x=self.robot.AXIS_OFFSET_X, z=0.3).material('#C0C0C0', 1.0)

    def update(self) -> None:
        super().update()
        y_relative_position = self.robot.steps_to_linear(self.robot.yaxis_home_position + self.robot.yaxis_position)
        z_relative_position = self.robot.steps_to_depth(self.robot.zaxis_home_position + self.robot.zaxis_position)
        self.tool.move(y=y_relative_position+self.robot.MAX_Y, z=z_relative_position)
