#!/usr/bin/evn python
# coding:utf-8

import serial
import sys
import time
import os
import datetime
import logging
import logging.config
from config import config


def build_log(model, suffix):
    log_name = localtime() + '.' + suffix
    file_path = os.path.join(
        os.path.expanduser('~'), 'UAVLog', model)
    if not os.path.exists(file_path):
        os.makedirs(file_path)
    return os.path.join(file_path, log_name)


def localtime():
    x = time.localtime(time.time())
    return time.strftime('%Y-%m-%d--%H:%M:%S', x)


def build_sbus():
    while True:
        try:
            com = serial.Serial(**config.sbus_serial)
            return com
        except serial.SerialException:
            info = sys.exc_info()
            print("{0}:{1}".format(*info))
            time.sleep(.5)


def config_logger(model, version):
    if version.find('alpha') != 1:
        LOG_FILE = build_log(model, 'log')
    else:
        LOG_FILE = ''
    LOGGING = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "simple": {
                # 'format': '%(asctime)s [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s',
                'format': '[%(levelname)s] [%(module)s:%(lineno)d] - %(message)s',
                'datefmt': '%Y-%m-%d %H:%M:%S'
            },
            'standard': {
                'format': '%(asctime)s [%(levelname)s]- %(message)s',
                # 'format': '%(asctime)s [%(threadName)s:%(thread)d] [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s',
                'datefmt': '%Y-%m-%d %H:%M:%S'
            },
        },

        "handlers": {
            "debugHandler": {
                "class": "logging.StreamHandler",
                "level": "DEBUG",
                "formatter": "simple",
                "stream": "ext://sys.stdout"
            },
            "infoHandler": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "simple",
                "stream": "ext://sys.stdout"
            },
            "errorHandler": {
                "class": "logging.StreamHandler",
                "level": "ERROR",
                "formatter": "simple",
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
                'handlers': ['errorHandler'],
                'level': 'WARNING',
            },
            "betaLogger": {
                'handlers': ['debugHandler'],
                # 'handlers': ['debugHandler', 'file'],
                'level': 'DEBUG',
                'propagate': False,
                'qualname': 'betaLogger'
            },
            "alphaLogger": {
                'handlers': ['infoHandler', 'file'],
                'level': 'INFO',
                'propagate': False,
                'qualname': 'alphaLogger'
            }
        },
    }
    return LOGGING


def init_logger(model):
    LOG_NAME = config.version + 'Logger'
    LOGGING = config_logger(model, LOG_NAME)
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
