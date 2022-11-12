#!/usr/bin/env python3
import rosys
from nicegui import ui

import hardware
import interface

is_real = rosys.hardware.SerialCommunication.is_possible()
if is_real:
    communication = rosys.hardware.SerialCommunication()
    robot_brain = hardware.RobotBrain(communication)
    robot = hardware.RobotHardware(robot_brain)
else:
    robot = hardware.RobotSimulation()
steerer = rosys.driving.Steerer(robot, speed_scaling=0.5)

odometer = rosys.driving.Odometer(robot)
driver = rosys.driving.Driver(robot, odometer)
driver.parameters.linear_speed_limit = 0.5
driver.parameters.angular_speed_limit = 0.5
automator = rosys.automation.Automator(robot, steerer)


@ui.page('/', shared=True)
async def index():
    interface.navigation_bar(robot)

ui.run()
