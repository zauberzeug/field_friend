from cmath import pi
from turtle import Shape
from nicegui.elements.scene_objects import Box, Cylinder, Group, Extrusion
from rosys.driving import Driver, Odometer
from rosys.driving import robot_object as RosysRobotObject
from rosys.geometry import Prism

from .robot_hardware import RobotHardware


class RobotObject(RosysRobotObject):

    def __init__(self, shape: Prism, odometer: Odometer, driver: Driver, robot: RobotHardware, *, debug: bool = False) -> None:
        super().__init__(shape, odometer)

        self.robot = robot
        with self:
            #with Group() as self.kranz:
                #Cylinder(0.2, 0.2, 0.45, 8, wireframe=True).material('#c0c0c0', opacity=0.5).move(0, 0, 0).rotate(pi/2, 0, 0) 
            with Group() as self.spirale:
                Box(width=0.05, height=0.05, depth=0.45).material('#3A3E42', 1.0)

    def update(self) -> None:
        super().update()

        if self.robot.swirl_is_down:
            self.spirale.move(z=-0.2)
        else:
            self.spirale.move(z=0.2)

                
                




        
