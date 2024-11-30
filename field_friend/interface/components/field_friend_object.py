from __future__ import annotations

import numpy as np
from nicegui import ui
from nicegui.elements.scene_objects import Box, Cylinder, Extrusion, Group, Stl
from rosys import config
from rosys.geometry import Prism
from rosys.vision import CameraProjector, CameraProvider, camera_objects

from ...hardware import Axis, ChainAxis, FieldFriend, Tornado
from ...robot_locator import RobotLocator


class RobotObject(Group):
    """The RobotObject UI element displays the robot with its given shape in a 3D scene.

    The current pose is taken from a given robot_locator.
    The `debug` argument can be set to show a wireframe instead of a closed polygon.
    """

    def __init__(self, shape: Prism, robot_locator: RobotLocator, *, debug: bool = False) -> None:
        super().__init__()
        self.shape = shape
        self.robot_locator = robot_locator
        self.robot_object: Extrusion | Stl
        with self:
            outline = [list(point) for point in self.shape.outline]
            self.robot_object = Extrusion(outline, self.shape.height, wireframe=debug)
            self.robot_object.material('#4488ff', 0.5)
        ui.timer(config.ui_update_interval, self.update)

    def with_stl(self, url: str, *,
                 x: float = 0, y: float = 0, z: float = 0,
                 omega: float = 0, phi: float = 0, kappa: float = 0,
                 scale: float = 1.0,
                 color: str = '#ffffff', opacity: float = 1.0) -> RobotObject:
        """Sets an STL to be displayed as the robot.

        The file can be served from a local directory with [app.add_static_files(url, path)](https://nicegui.io/reference#static_files).
        """
        self.robot_object.delete()
        with self:
            self.robot_object = Stl(url).move(x, y, z).rotate(omega, phi, kappa).scale(scale).material(color, opacity)
        return self

    def update(self) -> None:
        self.move(self.robot_locator.pose.x, self.robot_locator.pose.y)
        self.rotate(0, 0, self.robot_locator.pose.yaw)


class FieldFriendObject(RobotObject):

    def __init__(self, robot_locator: RobotLocator, camera_provider: CameraProvider,
                 field_friend: FieldFriend) -> None:
        super().__init__(Prism(outline=[], height=0), robot_locator)

        self.robot = field_friend

        self.with_stl('assets/field_friend.stl', x=-0.365, y=-0.3, z=0.06, scale=0.001, color='#6E93D6', opacity=0.7)
        camera_objects(camera_provider, CameraProjector(camera_provider, interval=0.1), interval=0.1)
        with self:
            if isinstance(self.robot.y_axis, Axis):
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
        if isinstance(self.robot.y_axis, Axis) and isinstance(self.robot.z_axis, Axis):
            self.tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position, z=self.robot.z_axis.position)
            if self.robot.y_axis.position > self.robot.y_axis.max_position or self.robot.y_axis.position < self.robot.y_axis.min_position:
                self.tool.material('red')
            else:
                self.tool.material('#4488ff')
        elif isinstance(self.robot.y_axis, Axis) and isinstance(self.robot.z_axis, Tornado):
            self.tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position, z=self.robot.z_axis.position_z)
            if self.robot.y_axis.position > self.robot.y_axis.max_position or self.robot.y_axis.position < self.robot.y_axis.min_position:
                self.tool.material('red')
            else:
                self.tool.material('#4488ff')
        elif isinstance(self.robot.y_axis, ChainAxis):
            if self.robot.y_axis.MIN_POSITION <= self.robot.y_axis.position <= self.robot.y_axis.MAX_POSITION:
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.position)
                # TODO: what to do if z_axis is not an Axis?
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.position,
                                      z=self.robot.z_axis.position)  # type: ignore
            elif self.robot.y_axis.position > self.robot.y_axis.MAX_POSITION:
                difference = self.robot.y_axis.position - self.robot.y_axis.MAX_POSITION
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.MAX_POSITION - difference)
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.MAX_POSITION - difference)
            else:
                difference = self.robot.y_axis.MIN_POSITION - self.robot.y_axis.position
                self.tool.move(x=self.robot.WORK_X_CHOP, y=self.robot.y_axis.MIN_POSITION + difference)
                self.second_tool.move(x=self.robot.WORK_X, y=self.robot.y_axis.MIN_POSITION + difference)
