import logging
import os
import sys
from pathlib import Path

import icecream

project = 'field_friend'

PATH = Path('~/.rosys').expanduser()


class PackagePathFilter(logging.Filter):
    # https://stackoverflow.com/a/52582536/3419103
    def filter(self, record):
        pathname = record.pathname
        record.relativepath = None
        abs_sys_paths = map(os.path.abspath, sys.path)
        for path in sorted(abs_sys_paths, key=len, reverse=True):  # longer paths first
            if not path.endswith(os.sep):
                path += os.sep
            if pathname.startswith(path):
                record.relativepath = os.path.relpath(pathname, path)
                break
        return True


def configure():
    icecream.install()

    PATH.mkdir(parents=True, exist_ok=True)

    config = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                'format': '%(asctime)s.%(msecs)03d [%(levelname)s] %(relativepath)s:%(lineno)d: %(message)s',
                'datefmt': '%Y-%m-%d %H:%M:%S',
            },
        },
        'filters': {
            'package_path_filter': {
                '()': PackagePathFilter,
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
                'level': 'DEBUG',
                'propagate': False,
            },
            'asyncio': {
                'handlers': ['debugfile'],
                'level': 'WARNING',
                'propagate': False,
            },
            'rosys': {
                'handlers': ['console', 'debugfile'],
                'level': 'DEBUG',
                'propagate': False,
            },
            project: {
                'handlers': ['console', 'debugfile'],
                'level': 'DEBUG',
                'propagate': False,
            },
        },
    }

    logging.config.dictConfig(config)
    return logging.getLogger(project)
