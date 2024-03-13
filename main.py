#!/usr/bin/env python3
from nicegui import app, ui

import field_friend.log_configuration as log_configuration
from field_friend import interface
from field_friend.system import System

logger = log_configuration.configure()
app.add_static_files('/assets', 'assets')


def startup() -> None:
    system = System()

    # /
    interface.pages.main_page(system)
    # /field
    interface.pages.field_planner_page(system)
    # /path
    interface.pages.path_planner_page(system)
    # /test
    interface.pages.test_page(system)
    # /kpis
    interface.pages.kpi_page(system)

    # /dev
    @ui.page('/dev')
    def dev_page():
        main_page = interface.pages.main_page(system, dev=True)
        main_page.content()

    # /status
    @app.get('/status')
    def status():
        return {'status': 'ok'}


app.on_startup(startup)

ui.run(title='Field Friend', port=80, favicon='assets/favicon.ico')
