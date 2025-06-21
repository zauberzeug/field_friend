#!/usr/bin/env python3
import logging
import os

from dotenv import load_dotenv
from nicegui import app, ui
from rosys.analysis import logging_page, videos_page

from field_friend import api, interface, log_configuration
from field_friend.interface.components.log_monitor import LogMonitor
from field_friend.system import System

logger = log_configuration.configure()
app.add_static_files('/assets', 'assets')

if os.environ.get('ROBOT_ID'):
    logging.warning(f'The ROBOT_ID environment variable is set to {os.environ.get("ROBOT_ID")} '
                    'and takes precedence over anything in the .env file.')

load_dotenv('.env')


def startup() -> None:
    robot_id = os.environ.get('ROBOT_ID')
    if robot_id is None:
        msg = 'The ROBOT_ID environment variable is not set. Please set it in the .env file.'
        logging.warning(msg)
        ui.label(msg).classes('text-xl')
        return
    robot_id = robot_id.lower()
    logger.info('Starting Field Friend for robot %s', robot_id)
    system = System(robot_id).persistent()

    log_monitor = LogMonitor().persistent(key='field_friend.log_monitor')
    interface.main_page(system)  # /
    interface.dev_page(system, log_monitor)  # /dev
    interface.monitor_page(system)  # /monitor
    interface.bms_page(system)  # /bms
    interface.low_bandwidth_page(system, log_monitor)  # /lb
    logging_page(['field_friend', 'rosys'])  # /logging
    videos_page()  # /videos

    # API Endpoints
    api.Online()  # get /api/online
    api.Status(system)  # get /api/status
    api.Position(system)  # get /api/position
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
