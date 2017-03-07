#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import os
import logging
import logging.config
import colorlog
from lib.config import config


def localtime():
    x = time.localtime(time.time())
    return time.strftime('%Y-%m-%d--%H:%M:%S', x)


def build_log(model, suffix):
    log_name = localtime() + '.' + suffix
    file_path = os.path.join(
        os.path.expanduser('~'), 'UAVLog', model)
    if not os.path.exists(file_path):
        os.makedirs(file_path)
    return os.path.join(file_path, log_name)


def config_logger(model):
    LOG_FILE = build_log(model, 'log')

    LOGGING = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            'colored': {
                '()': 'colorlog.ColoredFormatter',
                'format': "%(log_color)s[%(levelname)-8s] %(white)s%(module)-9s:%(lineno)-3d %(cyan)s%(message)s ",
                'datefmt': '%Y-%m-%d %H:%M:%S'
            },
            "simple": {
                # 'format': '%(asctime)s [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s',
                'format': '[%(levelname)s] [%(module)s:%(lineno)d] - %(message)s',
                'datefmt': '%Y-%m-%d %H:%M:%S'
            },
        },

        "handlers": {
            "debugHandler": {
                "class": "logging.StreamHandler",
                "level": "DEBUG",
                "formatter": "colored",
                "stream": "ext://sys.stdout"
            },
            "infoHandler": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "colored",
                "stream": "ext://sys.stdout"
            },
            "file": {
                "class": "logging.handlers.RotatingFileHandler",
                "level": "DEBUG",
                "formatter": "simple",
                "filename": LOG_FILE,
                'mode': 'w+',
                "maxBytes": 1024 * 1024 * 5,  # 5 MB
                "backupCount": 20,
                "encoding": "utf8"
            },
        },
        "loggers": {
            "root": {
                'handlers': ['infoHandler'],
                'level': 'WARNING',
            },
            "beta": {
                # 'handlers': ['debugHandler'],
                'handlers': ['debugHandler', 'file'],
                'level': 'DEBUG',
                'propagate': False,
                'qualname': 'beta'
            },
            "alpha": {
                'handlers': ['infoHandler', 'file'],
                'level': 'INFO',
                'propagate': False,
                'qualname': 'alpha'
            }
        },
    }
    return LOGGING


def init_logger(model):
    LOG_NAME = config.version
    LOGGING = config_logger(model)
    logging.config.dictConfig(LOGGING)
    logger = logging.getLogger(LOG_NAME)
    return logger

model = config.drone['Model']
logger = init_logger(model)

if __name__ == '__main__':
    logger.debug('debug message')
    logger.info('info message')
    logger.warn('warn message')
    logger.error('error message')
    logger.critical('critical message')
