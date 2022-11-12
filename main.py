#!/usr/bin/env python3
import rosys
from nicegui import ui

import hardware
import interface

try:
    robot = hardware.RobotHardware()
except:
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

    with ui.column().classes('w-full no-wrap items-stretch q-px-md'):
        interface.operation(steerer, automator, odometer)
ui.run()
