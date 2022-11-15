#!/usr/bin/env python3
import rosys
from nicegui import ui

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
else:
    robot = hardware.RobotSimulation()
    usb_camera_provider = rosys.vision.UsbCameraProviderSimulation()
steerer = rosys.driving.Steerer(robot, speed_scaling=0.5)
odometer = rosys.driving.Odometer(robot)
driver = rosys.driving.Driver(robot, odometer)
driver.parameters.linear_speed_limit = 0.5
driver.parameters.angular_speed_limit = 0.5
automator = rosys.automation.Automator(robot, steerer)


async def start_homing() -> None:
    await robot.try_reference_yaxis()


@ui.page('/', shared=True)
async def index():
    ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
    interface.navigation_bar(robot)

    with ui.row().classes('fit items-stretch justify-around').style('flex-wrap:nowrap'):
        interface.operation(steerer, automator, odometer, usb_camera_provider)
        interface.camera(usb_camera_provider)
        with ui.card():
            if robot.is_real:
                robot.robot_brain.developer_ui()
                robot.robot_brain.communication.debug_ui()
                with ui.row():
                    with ui.menu() as menu:
                        ui.menu_item('perform homing', on_click=start_homing)
                    ui.button(on_click=menu.open).props('dense fab-mini outline icon=more_vert')
                rosys.on_startup(robot.robot_brain.lizard_firmware.ensure_lizard_version)
            else:
                rosys.simulation_ui()

if robot.is_simulation:
    rosys.on_startup(lambda: hardware.simulation.create_weedcam(usb_camera_provider))

ui.run(title='Field Friend', port=80 if robot.is_real else 8080)
