#!/usr/bin/env python3
import os

from dotenv import load_dotenv
from nicegui import app, ui
from rosys.analysis import logging_page, videos_page

from field_friend import interface, log_configuration
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
    logger.info('Starting Field Friend for robot %s', robot_id)
    System.version = os.environ.get('VERSION') or robot_id
    System.robot_id = robot_id
    system = System()

    interface.main_page(system)  # /
    interface.dev_page(system)  # /dev
    interface.monitor_page(system)  # /monitor
    interface.bms_page(system)  # /bms

    @app.get('/status')  # /status
    def status():
        return {'status': 'ok'}

    logging_page(['field_friend', 'rosys'])  # /logging
    videos_page()  # /videos


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
