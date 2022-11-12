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
    ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
    interface.navigation_bar(robot)

    with ui.column().classes('w-full no-wrap items-stretch q-px-md'):
        with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
            interface.operation(steerer, automator, odometer)
ui.run()
