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
    detector = rosys.vision.DetectorHardware(port=8004)
else:
    robot = hardware.RobotSimulation()
    usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
    detector = rosys.vision.DetectorSimulation(usb_camera_provider)
camera_selector = hardware.CameraSelector(usb_camera_provider)
plant_provider = automations.PlantProvider()
steerer = rosys.driving.Steerer(robot, speed_scaling=0.2)
odometer = rosys.driving.Odometer(robot)
driver = rosys.driving.Driver(robot, odometer)
driver.parameters.linear_speed_limit = 0.3
driver.parameters.angular_speed_limit = 0.3
driver.parameters.can_drive_backwards = False
automator = rosys.automation.Automator(robot, steerer)
weeding = automations.Weeding(robot, driver, detector, camera_selector, plant_provider)
automator.default_automation = weeding.start


@ui.page('/', shared=True)
async def index():
    ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
    interface.navigation_bar(robot)

    with ui.row().classes('fit items-stretch justify-around').style('flex-wrap:nowrap'):
        interface.operation(robot, steerer, automator, odometer, usb_camera_provider)
        interface.camera(camera_selector, usb_camera_provider, automator, robot, detector, weeding)
    interface.development(robot, automator)

if robot.is_simulation:
    rosys.on_startup(lambda: hardware.simulation.create_weedcam(usb_camera_provider))

ui.run(title='Field Friend', port=80 if robot.is_real else 8080)
