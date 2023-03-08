from nicegui import app
from nicegui.elements.scene_objects import Box, Group
from rosys.driving import Odometer, robot_object
from rosys.geometry import Prism
from rosys.vision import CameraProjector, CameraProvider, camera_objects

from ..hardware import FieldFriend


class robot_object(robot_object):
    width = 0.63
    length = 0.78
    offset = 0.16
    chain_width = 0.145

    shape = Prism(
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

    def __init__(self, odometer: Odometer, camera_provider: CameraProvider,
                 field_friend: FieldFriend) -> None:
        super().__init__(self.shape, odometer, debug=True)

        self.y_axis = field_friend.y_axis
        self.z_axis = field_friend.z_axis
        self.odometer = odometer

        app.add_static_files('/assets', 'assets')
        self.with_stl('assets/field_friend.stl', x=-0.15, y=-0.3, z=0.05, scale=0.001, color='#6E93D6')
        with self:
            with Group() as self.camera:
                camera_objects(camera_provider, CameraProjector(camera_provider))
            if self.y_axis is not None:
                with Group() as self.axis:
                    Box(width=0.05, height=0.63, depth=0.08).move(
                        x=self.y_axis.AXIS_OFFSET_X+0.025, z=0.34).material('#6E93D6', 1.0)
            if self.z_axis is not None:
                with Group() as self.tool:
                    Box(width=0.02, height=0.02, depth=0.3).move(
                        x=self.y_axis.AXIS_OFFSET_X, z=0.3).material('#C0C0C0', 1.0)

    def update(self) -> None:
        super().update()
        if self.y_axis and self.z_axis is not None:
            y_relative_position = self.y_axis.steps_to_linear(
                self.y_axis.home_position + self.y_axis.position)
            z_relative_position = self.z_axis.steps_to_depth(
                self.z_axis.home_position + self.z_axis.position)
            self.tool.move(y=y_relative_position+self.y_axis.MAX_Y, z=z_relative_position)
