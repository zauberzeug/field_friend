#!/usr/bin/env python3
from nicegui import app, ui

import log
from field_friend import interface
from system import System

log = log.configure()
app.add_static_files('/assets', 'assets')


def startup() -> None:
    system = System()

    @ui.page('/')
    def page() -> None:
        ui.colors(primary='#6E93D6', secondary='#53B689', accent='#111B1E', positive='#53B689')
        status_drawer = interface.status_drawer(system.field_friend, system.gnss, system.odometer)
        interface.header_bar(system, status_drawer)
        interface.system_bar()
        with ui.column().classes('w-full items-stretch'):
            with ui.row().classes('items-stretch justify-items-stretch').style('flex-wrap:nowrap'):
                interface.operation(
                    system.field_friend, system.steerer, system.automator, system.odometer, system.usb_camera_provider,
                    system.plant_provider, system.plant_detector, system.puncher, system.weeding)
                interface.cameras(system.camera_selector, system.usb_camera_provider,
                                  system.automator, system.detector, system.puncher)
            with ui.row().classes('items-stretch justify-items-stretch'):
                interface.development(system.field_friend)
                interface.axis_control(system.field_friend, system.automator, system.puncher)

    @app.get('/status')
    def status():
        return {'status': 'ok'}


app.on_startup(startup)

ui.run(title='Field Friend', port=80)
