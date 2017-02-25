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
    portname = '/dev/sbus'
    # portname = '/dev/ttyAMA0'
    while True:
        try:
            com = serial.Serial(port=portname,
                                baudrate=100000,
                                parity=serial.PARITY_EVEN,
                                stopbits=serial.STOPBITS_TWO,
                                bytesize=serial.EIGHTBITS)
            return com
        except serial.SerialException:
            info = sys.exc_info()
            _log("{0}:{1}".format(*info))
            time.sleep(.5)


def config_logger(model, version):
    if version.find('release') != 1:
        LOG_FILE = build_log(model, 'log')
    else:
        LOG_FILE = ''
    LOGGING = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "simple": {
                # 'format': '%(asctime)s [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s',
                'format': '[%(levelname)s]- %(message)s',
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
            "debugLogger": {
                'handlers': ['debugHandler'],
                # 'handlers': ['debugHandler', 'file'],
                'level': 'DEBUG',
                'propagate': False,
                'qualname': 'debugLogger'
            },
            "releaseLogger": {
                'handlers': ['infoHandler', 'file'],
                'level': 'INFO',
                'propagate': False,
                'qualname': 'infoLogger'
            }
        },
    }
    return LOGGING


def init_logger(model):
    LOG_NAME = 'debugLogger'
    LOGGING = config_logger(model, LOG_NAME)
    logging.config.dictConfig(LOGGING)
    logger = logging.getLogger(LOG_NAME)
    return logger


def _log(message):
    global logger
    logger.info(message)
    # print(message)


def _info(message):
    global logger
    logger.info(message)
    # print(message)


def _debug(message):
    global logger
    logger.debug(message)
    # print(message)


def _error(message):
    global logger
    logger.error(message)
    # print(message)


def _critical(msg):
    global logger
    self.logger.critical(msg)


def exe_cmd(vehicle, command):
    flag = True
    print 'Recv--', command
    input = {'STOP': {}, 'FORWARD': {'ELE': 1},
             'BACKWARD': {'ELE': -1},
             'LEFT_YAW': {'RUD': -1}, 'RIGHT_YAW': {'RUD': 1},
             'LEFT_ROLL': {'AIL': -1}, 'RIGHT_ROLL': {'AIL': 1},
             'UP': {'THR': 1}, 'DOWN': {'THR': -1}}

    cmds = command.split(' ')
    action = {}
    for cmd in cmds:
        if cmd in input and cmd not in action:
            action = dict(action, **input[cmd])
        else:
            vehicle._error('Command({}) is unvalid!'.format(command))
            action = {}
            flag = False
            break

    # print 'action:', action
    vehicle.control_FRU(**action)
    return flag

model = config.model
logger = init_logger(model)

if __name__ == '__main__':
    logger.debug('debug message')
    logger.info('info message')
    logger.warn('warn message')
    logger.error('error message')
    logger.critical('critical message')
