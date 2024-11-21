import logging
from pathlib import Path

import icecream
from rosys import helpers

project = 'field_friend'

PATH = Path('~/.rosys').expanduser()


def configure():
    icecream.install()

    PATH.mkdir(parents=True, exist_ok=True)

    config = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
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
                'maxBytes': 1024 * 1000 * 10,  # each file max 10 mb
                'backupCount': 10  # max 100 mb of logs
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
            }
        },
        'loggers': {
            '': {  # root logger
                'handlers': ['console'],
                'level': 'WARN',
                'propagate': False,
            },
            'rosys.communication': {
                'handlers': ['communicationfile'],
                'level': 'INFO',
                'propagate': False,
            },
            'field_friend.bms': {
                'handlers': ['batteryfile'],
                'level': 'INFO',
                'propagate': False,
            },
            'asyncio': {
                'handlers': ['debugfile'],
                'level': 'WARNING',
                'propagate': False,
            },
            'rosys': {
                'handlers': ['console', 'debugfile'],
                'level': 'INFO',
                'propagate': False,
            },
            project: {
                'handlers': ['console', 'debugfile'],
                'level': 'INFO',
                'propagate': False,
            },
        },
    }

    logging.config.dictConfig(config)
    return logging.getLogger(project)
