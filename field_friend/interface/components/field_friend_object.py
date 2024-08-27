import numpy as np
from nicegui.elements.scene_objects import Box, Cylinder, Extrusion, Group
from rosys.driving import Odometer, robot_object
from rosys.geometry import Prism
from rosys.vision import CameraProjector, CameraProvider, camera_objects

from ...hardware import ChainAxis, FieldFriend, Tornado, YAxis, ZAxis


class field_friend_object(robot_object):

    def __init__(self, odometer: Odometer, camera_provider: CameraProvider,
                 field_friend: FieldFriend) -> None:
        super().__init__(Prism(outline=[], height=0), odometer)

        self.odometer = odometer
        self.robot = field_friend

        self.with_stl('assets/field_friend.stl', x=-0.365, y=-0.3, z=0.06, scale=0.001, color='#6E93D6', opacity=0.7)
        camera_objects(camera_provider, CameraProjector(camera_provider))
        with self:
            if isinstance(self.robot.y_axis, YAxis):
                with Group() as self.tool:
                    Box(0.015, 0.015, 0.35).material('#4488ff').move(z=0.4)
                    Cylinder(0.03, 0, 0.05).material('#4488ff').move(z=0.2).rotate(1.571, 0, 0)
            elif isinstance(self.robot.y_axis, ChainAxis):
                with Group() as self.tool:
                    Box(0.03, 0.03, 0.35).material('#4488ff').move(z=0.25)
                    Extrusion([[0, 0], [0.10, 0], [0, 0.10]], 0.04).material(
                        '#4488ff').move(z=0.1, y=-0.02).rotate(-np.pi/2, np.pi/4, 0)
                with Group() as self.second_tool:
                    Box(0.02, 0.02, 0.35).material('#4488ff').move(z=0.3)
                    Cylinder(0.03, 0, 0.05).material('#4488ff').move(z=0.1).rotate(np.pi/2, 0, 0)

    def update(self) -> None:
        super().update()
        if isinstance(self.robot.y_axis, YAxis) and isinstance(self.robot.z_axis, ZAxis):
            self.tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position, z=self.robot.z_axis.position)
            if self.robot.y_axis.position > self.robot.y_axis.max_position or self.robot.y_axis.position < self.robot.y_axis.min_position:
                self.tool.material('red')
            else:
                self.tool.material('#4488ff')
        elif isinstance(self.robot.y_axis, YAxis) and isinstance(self.robot.z_axis, Tornado):
            self.tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position, z=self.robot.z_axis.position_z)
            if self.robot.y_axis.position > self.robot.y_axis.max_position or self.robot.y_axis.position < self.robot.y_axis.min_position:
                self.tool.material('red')
            else:
                self.tool.material('#4488ff')
        elif isinstance(self.robot.y_axis, ChainAxis):
            if self.robot.y_axis.MIN_POSITION <= self.robot.y_axis.position <= self.robot.y_axis.MAX_POSITION:
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.position)
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position,
                                      z=self.robot.z_axis.position)
            elif self.robot.y_axis.position > self.robot.y_axis.MAX_POSITION:
                difference = self.robot.y_axis.position - self.robot.y_axis.MAX_POSITION
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.MAX_POSITION - difference)
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.MAX_POSITION - difference)
            else:
                difference = self.robot.y_axis.MIN_POSITION - self.robot.y_axis.position
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.MIN_POSITION + difference)
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.MIN_POSITION + difference)
