#!/usr/bin/env python3
import os

from dotenv import load_dotenv
from nicegui import app, ui
from rosys.analysis import logging_page

import field_friend.log_configuration as log_configuration
from field_friend import interface
from field_friend.interface.components import header_bar, status_drawer, system_bar
from field_friend.system import System

logger = log_configuration.configure()
app.add_static_files('/assets', 'assets')


load_dotenv('.env')


def startup() -> None:
    robot_id = os.environ.get('ROBOT_ID')
    if robot_id is None:
        msg = 'The ROBOT_ID environment variable is not set. Please set it in the .env file.'
        logger.warning(msg)
        ui.label(msg).classes('text-xl')
        return
    logger.info(f'Starting Field Friend for robot {robot_id}')
    System.version = os.environ.get('VERSION') or robot_id
    system = System()

    def page_wrapper() -> None:
        drawer = status_drawer(system, system.field_friend, system.gnss, system.odometer, system.automator)
        header_bar(system, drawer)
        system_bar()

    interface.pages.main_page(page_wrapper, system)  # /
    interface.pages.field_planner_page(page_wrapper, system)  # /field
    # interface.pages.path_planner_page(page_wrapper, system)  # /path
    interface.pages.dev_page(page_wrapper, system)  # /dev
    interface.pages.test_page(page_wrapper, system)  # /test
    interface.pages.kpi_page(page_wrapper, system)  # /kpis
    interface.pages.monitor_page(page_wrapper, system)  # /monitor
    interface.pages.io_page(page_wrapper, system)  # /monitor

    @app.get('/status')  # /status
    def status():
        return {'status': 'ok'}

    logging_page(['field_friend', 'rosys'])  # /logging


app.on_startup(startup)

ui.run(
    title='Field Friend',
    port=int(os.environ.get('PORT', '80')),
    storage_secret='feldfreund',
    favicon='assets/favicon.ico',
    binding_refresh_interval=0.3,
    reconnect_timeout=10,
    on_air=os.environ.get('ON_AIR_TOKEN'),
)
