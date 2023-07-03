from nicegui.elements.scene_objects import Box, Cylinder, Group
from rosys.driving import Odometer, robot_object
from rosys.geometry import Prism
from rosys.vision import CameraProjector, CameraProvider, camera_objects

from ..hardware import FieldFriend


class robot_object(robot_object):

    def __init__(self, odometer: Odometer, camera_provider: CameraProvider,
                 field_friend: FieldFriend) -> None:
        super().__init__(Prism(outline=[], height=0), odometer)

        self.odometer = odometer
        self.robot = field_friend

        self.with_stl('assets/field_friend.stl', x=-0.365, y=-0.3, z=0.06, scale=0.001, color='#6E93D6', opacity=0.7)
        with self:
            camera_objects(camera_provider, CameraProjector(camera_provider))
            if self.robot.y_axis is not None and self.robot.z_axis is not None:
                with Group() as self.tool:
                    Box(0.03, 0.03, 0.35).material('#4488ff').move(z=0.4)
                    Cylinder(0.05, 0, 0.05).material('#4488ff').move(z=0.2).rotate(1.571, 0, 0)

    def update(self) -> None:
        super().update()
        if self.robot.y_axis and self.robot.z_axis is not None:
            self.tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position, z=-self.robot.z_axis.depth)
