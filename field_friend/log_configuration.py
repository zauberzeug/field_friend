import atexit
import logging.config
from logging.handlers import QueueListener
from pathlib import Path
from queue import Queue

import coloredlogs
import icecream
from rosys import helpers

project = 'field_friend'
PATH = Path('~/.rosys').expanduser()


def configure():
    icecream.install()
    PATH.mkdir(parents=True, exist_ok=True)

    config = {
        'version': 1,
        'disable_existing_loggers': False,
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
            'debug_queue': {
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
                'handlers': ['debugfile', 'console'],
            },
            project: {
                'level': 'INFO',
                'propagate': True,
            },
            'rosys': {
                'level': 'INFO',
                'propagate': True,
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

    return logging.getLogger(project)
