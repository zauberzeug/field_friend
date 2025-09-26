import atexit
import logging.config
from logging.handlers import QueueListener
from pathlib import Path
from queue import Queue
from typing import Any

import coloredlogs
import icecream
from rosys import helpers

project = 'field_friend'
PATH = Path('~/.rosys').expanduser()


def configure():
    icecream.install()
    PATH.mkdir(parents=True, exist_ok=True)

    config: dict[str, Any] = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                '()': coloredlogs.ColoredFormatter,
                'format': r'%(asctime)s.%(msecs)03d [%(levelname)s] %(relative_path)s:%(lineno)d: %(message)s',
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
                'level': 'INFO',
                'stream': 'ext://sys.stdout'
            },
            'debugfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'debug.log',
                'maxBytes': 1024 * 1000 * 10,
                'backupCount': 10
            },
            'communicationfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'communication.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 50  # max 500 mb of logs
            },
            'batteryfile': {
                'level': 'DEBUG',
                'class': 'logging.handlers.RotatingFileHandler',
                'formatter': 'default',
                'filters': ['package_path_filter'],
                'filename': PATH / 'battery.log',
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 50  # max 500 mb of logs
            },
            'debug_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
            'lizard_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
            'battery_queue': {
                'class': 'logging.handlers.QueueHandler',
                'queue': Queue(),
            },
        },
        'loggers': {
            '': {  # root logger
                'handlers': ['debug_queue'],
                'level': 'WARN',
                'propagate': False,
            },
            'DUMMY': {
                # to be able to access these handlers below
                'handlers': ['console', 'debug_queue', 'debugfile', 'lizard_queue', 'communicationfile', 'battery_queue', 'batteryfile'],
            },
            'rosys': {
                'handlers': ['debug_queue'],
                'level': 'INFO',
                'propagate': False,
            },
            'nicegui': {
                'handlers': ['debug_queue'],
                'level': 'INFO',
                'propagate': False,
            },
            'rosys.communication': {
                'handlers': ['lizard_queue'],
                'level': 'INFO',
                'propagate': False,
            },
            project: {
                'handlers': ['debug_queue'],
                'level': 'INFO',
                'propagate': False,
            },
            f'{project}.bms': {
                'handlers': ['battery_queue'],
                'level': 'INFO',
                'propagate': False,
            },
        },
    }

    logging.config.dictConfig(config)

    # Setup queue listeners
    handlers = {h.name: h for h in logging.getLogger('DUMMY').handlers}

    debug_queue = config['handlers']['debug_queue']['queue']
    debug_listener = QueueListener(debug_queue, handlers['debugfile'], handlers['console'])
    debug_listener.start()
    atexit.register(debug_listener.stop)

    lizard_queue = config['handlers']['lizard_queue']['queue']
    lizard_listener = QueueListener(lizard_queue, handlers['communicationfile'])
    lizard_listener.start()
    atexit.register(lizard_listener.stop)

    battery_queue = config['handlers']['battery_queue']['queue']
    battery_listener = QueueListener(battery_queue, handlers['batteryfile'])
    battery_listener.start()
    atexit.register(battery_listener.stop)

    return logging.getLogger(project)
