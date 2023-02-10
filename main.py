#!/usr/bin/env python3
from nicegui import app, ui

import log
from field_friend import interface
from system import System

log = log.configure()


def startup() -> None:
    system = System()

    @ui.page('/')
    async def main_page():
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        interface.navigation_bar(system.robot, system.estop)

        with ui.row().classes('fit items-stretch justify-around').style('flex-wrap:nowrap'):
            interface.operation(system.robot, system.steerer, system.automator, system.odometer,
                                system.usb_camera_provider, system.plant_provider)
            interface.camera(system.camera_selector, system.usb_camera_provider,
                             system.automator, system.robot, system.detector, system.weeding)
        interface.development(system.robot, system.automator)


app.on_startup(startup)

ui.run(title='Field Friend', port=80)
