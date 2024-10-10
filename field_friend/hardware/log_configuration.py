import atexit
import logging.config
from logging.handlers import QueueListener
from pathlib import Path
from queue import Queue

import coloredlogs
import icecream
from rosys import helpers

PATH = Path('~/.rosys').expanduser()


def setup():
    icecream.install()

    PATH.mkdir(parents=True, exist_ok=True)

    config = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                '()': coloredlogs.ColoredFormatter,
                'format': r'%(asctime)s.%(msecs)03d [%(levelname)s] %(relative_path)s:%(lineno)d: %(message)s',
                'datefmt': r'%Y-%m-%d %H:%M:%S',
            },
            'fast': {
                'format': r'%(asctime)s.%(msecs)03d: %(message)s',
                'datefmt': r'%Y-%m-%d %H:%M:%S',
            },
        },
        'filters': {
            'package_path_filter': {
                '()': helpers.PackagePathFilter,
            },
        },
        'handlers': {
            'console': {
                'class': 'logging.StreamHandler',
                'filters': ['package_path_filter'],
                'formatter': 'default',
                'level': 'DEBUG',
                'stream': 'ext://sys.stdout'
            },
            'debugfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'debug.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
            },
            'communicationfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'fast',
                'filename': PATH / 'communication.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 50  # max 500 mb of logs
            },
            'memoryfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'memory.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
            },
            'locatorfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'locator.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
            },
            'pathplanningfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'pathplanning.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
            },
            'batteryfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'battery.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
            },
            'onairfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'fast',
                'filename': PATH / 'on_air.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 50  # max 500 mb of logs
            },
            'debug_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
                # 'respect_handler_level': True,
            },
            'communication_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
                # 'respect_handler_level': True,
            },
            'pathplanning_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
            'locator_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
            'oit_locator_info_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
                'level': 'INFO',
            },
            'battery_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
            'on_air_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
        },
        'loggers': {
            '': {  # root logger
                'handlers': ['debug_queue'],
                'level': 'INFO',
            },
            'DUMMY': {
                # to be able to access these handlers below
                'handlers': ['debugfile', 'console', 'communicationfile', 'pathplanningfile', 'locatorfile', 'batteryfile', 'onairfile'],
            },
            'httpx': {
                'level': 'WARNING',
            },
            'rosys.communication': {
                'handlers': ['communication_queue'],
                'propagate': False,
            },
            'rosys.analysis.memory': {
                'handlers': ['memoryfile'],
                'propagate': False,
            },
            'rosys.pathplanning.PlannerProcess': {
                'handlers': ['pathplanning_queue'],
                'propagate': False,
            },
            'outside_in_tracking.robot_locator': {
                'handlers': ['locator_queue', 'oit_locator_info_queue'],
                'level': 'DEBUG',
                'propagate': False,
            },
            'active_cleaner.bms': {
                'handlers': ['battery_queue'],
                'propagate': True,
            },
            'nicegui.air': {
                'handlers': ['on_air_queue'],
                'level': 'DEBUG',
                'propagate': False,
            }
        },
    }

    logging.config.dictConfig(config)

    handlers = {h.name: h for h in logging.getLogger('DUMMY').handlers}

    debug_queue = config['handlers']['debug_queue']['queue']
    debug_listener = QueueListener(debug_queue, handlers['debugfile'], handlers['console'])
    debug_listener.start()
    atexit.register(debug_listener.stop)

    communication_queue = config['handlers']['communication_queue']['queue']
    communication_listener = QueueListener(communication_queue, handlers['communicationfile'])
    communication_listener.start()
    atexit.register(communication_listener.stop)

    pathplanning_queue = config['handlers']['pathplanning_queue']['queue']
    pathplanning_listener = QueueListener(pathplanning_queue, handlers['pathplanningfile'])
    pathplanning_listener.start()
    atexit.register(pathplanning_listener.stop)

    locator_queue = config['handlers']['locator_queue']['queue']
    locator_listener = QueueListener(locator_queue, handlers['locatorfile'])
    locator_listener.start()
    atexit.register(locator_listener.stop)
    oit_locator_info_queue = config['handlers']['oit_locator_info_queue']['queue']
    oit_locator_info_listener = QueueListener(oit_locator_info_queue,
                                              handlers['debugfile'], handlers['console'])
    oit_locator_info_listener.start()
    atexit.register(oit_locator_info_listener.stop)

    battery_queue = config['handlers']['battery_queue']['queue']
    battery_listener = QueueListener(battery_queue, handlers['batteryfile'])
    battery_listener.start()
    atexit.register(battery_listener.stop)

    on_air_queue = config['handlers']['on_air_queue']['queue']
    on_air_listener = QueueListener(on_air_queue, handlers['onairfile'])
    on_air_listener.start()
    atexit.register(on_air_listener.stop)
