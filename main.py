#!/usr/bin/env python3
import logging
import os

from dotenv import load_dotenv
from nicegui import app, ui
from rosys.analysis import logging_page, videos_page

from field_friend import api, interface, log_configuration
from field_friend.system import System

logger = log_configuration.configure()
app.add_static_files('/assets', 'assets')


load_dotenv('.env')


def startup() -> None:
    robot_id = os.environ.get('ROBOT_ID')
    if robot_id is None:
        msg = 'The ROBOT_ID environment variable is not set. Please set it in the .env file.'
        logging.warning(msg)
        ui.label(msg).classes('text-xl')
        return
    logger.info('Starting Field Friend for robot %s', robot_id)
    system = System(robot_id)

    interface.main_page(system)  # /
    interface.dev_page(system)  # /dev
    interface.monitor_page(system)  # /monitor
    interface.bms_page(system)  # /bms
    interface.low_bandwidth_page(system)  # /lb
    logging_page(['field_friend', 'rosys'])  # /logging
    videos_page()  # /videos
    interface.gnss_test_page(system)  # /gnss_test

    # API Endpoints
    api.Online()  # get /api/online
    api.Status(system)  # get /api/status
    api.Fields(system)  # get,post /api/fields
    api.Automation(system)  # get,post /api/automation/


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
