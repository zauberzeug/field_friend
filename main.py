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
        interface.navigation_bar(system.field_friend)

        with ui.row().classes('fit items-stretch justify-around').style('flex-wrap:nowrap'):
            interface.operation(system.field_friend, system.steerer, system.automator, system.odometer,
                                system.usb_camera_provider, system.plant_provider, system.puncher)
            interface.camera(system.camera_selector, system.usb_camera_provider,
                             system.automator, system.detector, system.puncher)
        interface.development(system.field_friend, system.automator, system.puncher)


app.on_startup(startup)

ui.run(title='Field Friend', port=80)
