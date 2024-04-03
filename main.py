#!/usr/bin/env python3
from nicegui import app, ui
from rosys.analysis import logging_page

import field_friend.log_configuration as log_configuration
from field_friend import interface
from field_friend.interface.components import header_bar, status_drawer, system_bar
from field_friend.system import System

logger = log_configuration.configure()
app.add_static_files('/assets', 'assets')


def startup() -> None:
    system = System()

    def page_wrapper() -> None:
        drawer = status_drawer(system, system.field_friend, system.gnss,
                               system.odometer, system.automator)
        header_bar(system, drawer)
        system_bar()

    # /
    interface.pages.main_page(page_wrapper, system, dev=False)
    # /field
    interface.pages.field_planner_page(page_wrapper, system)
    # /path
    interface.pages.path_planner_page(page_wrapper, system)
    # /dev
    interface.pages.dev_page(page_wrapper, system)
    # /test
    interface.pages.test_page(page_wrapper, system)
    # /kpis
    interface.pages.kpi_page(page_wrapper, system)
    # /monitor
    interface.pages.monitor_page(page_wrapper, system)
    # /status

    @app.get('/status')
    def status():
        return {'status': 'ok'}

    logging_page(['field_friend', 'rosys'])


app.on_startup(startup)

ui.run(title='Field Friend', port=80, favicon='assets/favicon.ico',
       binding_refresh_interval=0.3,
       reconnect_timeout=10)
