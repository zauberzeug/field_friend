#!/usr/bin/env python3
import rosys
from nicegui import ui

import automations
import hardware
import interface
import log

log = log.configure()

is_real = rosys.hardware.SerialCommunication.is_possible()
if is_real:
    communication = rosys.hardware.SerialCommunication()
    robot_brain = rosys.hardware.RobotBrain(communication)
    robot = hardware.RobotHardware(robot_brain)
    usb_camera_provider = rosys.vision.UsbCameraProviderHardware()
    detector = rosys.vision.DetectorHardware
else:
    robot = hardware.RobotSimulation()
    usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
    detector = rosys.vision.DetectorSimulation
coin_provider = automations.CoinProvider()
steerer = rosys.driving.Steerer(robot, speed_scaling=0.2)
odometer = rosys.driving.Odometer(robot)
driver = rosys.driving.Driver(robot, odometer)
driver.parameters.linear_speed_limit = 0.2
driver.parameters.angular_speed_limit = 0.2
automator = rosys.automation.Automator(robot, steerer)
coin_collecting = automations.CoinCollecting(robot, driver, detector, usb_camera_provider, coin_provider)
automator.default_automation = coin_collecting.start()


@ui.page('/', shared=True)
async def index():
    ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
    interface.navigation_bar(robot)

    with ui.row().classes('fit items-stretch justify-around').style('flex-wrap:nowrap'):
        interface.operation(steerer, automator, odometer, usb_camera_provider)
        interface.camera(usb_camera_provider, automator, robot, coin_collecting)
    interface.development(robot, automator)

if robot.is_simulation:
    rosys.on_startup(lambda: hardware.simulation.create_weedcam(usb_camera_provider))

ui.run(title='Field Friend', port=80 if robot.is_real else 8080)
