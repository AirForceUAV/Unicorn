#!/usr/bin/evn python
# coding:utf-8

import serial
import sys
import time
import os
import datetime
import logging
import logging.config

open_module = [
    'Sbus',
    'Compass',
    'GPS',
    # 'Baro',
    # 'IMU',
    # 'Lidar',
    'Cloud',
]

commands = [
    'arm()',
    'set_channels_mid()',
    'set_gear(2)',
    'yaw_left_brake()',
    'yaw_right_brake()',
    'roll_left_brake()',
    'roll_right_brake()',
    'forward_brake()',
    'backward_brake()',
    'up_brake()',
    # 'down_brake()',
    # 'condition_yaw(30)',
    # 'condition_yaw(300)',
    # 'set_target(-20, 0)',
    # 'Guided()',
    # 'download(0)',
    # 'Auto()',
    # 'disarm()',
    # 'GradualTHR(0, 60)'
]


def _log(message):
    print('>>> ' + message)


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
            _LOG("{0}:{1}".format(*info))
            time.sleep(.5)


def init_logger(model=None):
    LOG_NAME = 'console'
    model = model or 'Test'
    LOG_FILE = build_log(model, 'info')
    # print LOG_FILE
    LOGGING = {
        "version": 1,
        "disable_existing_loggers": False,
        "formatters": {
            "simple": {
                # 'format': '%(asctime)s [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s'
                'format': '%(asctime)s [%(name)s] [%(levelname)s]- %(message)s'
            },
            'standard': {
                'format': '%(asctime)s [%(threadName)s:%(thread)d] [%(name)s:%(lineno)d] [%(levelname)s]- %(message)s'
            },
        },

        "handlers": {
            "console": {
                "class": "logging.StreamHandler",
                "level": "DEBUG",
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

        # "loggers": {
        #     "app_name": {
        #         'handlers': ['console'],
        #         'level': 'DEBUG',
        #         'propagate': False
        #     }
        # },

        "root": {
            'handlers': ['console'],
            "level": "DEBUG",
            'propagate': False
        }
    }

    logging.config.dictConfig(LOGGING)
    logger = logging.getLogger(LOG_NAME)
    return logger


close_module = [
    'Sbus',
    'Compass',
    'GPS',
    'Baro',
    'IMU',
    'Lidar',
    'Cloud',
]

protobuf = {
    'Compass_State': True,
    'Sbus_State': True,
    'Sender_State': True,
    'Attitude': [
        -0.32,
        0.01,
        66],
    'Baro_State': True,
    'Pressure': 1013.25,
    'Temperature': 26,
    'ChannelsInput': [1000] * 8,
    'GPS_State': True,
    'Location': [
        36.11127966305683,
        116.2222,
        100],
    'NumStars': 16,
    'ChannelsOutput': [1000] * 8,
    'HomeLocation': [
        36.1111,
        116.2222],
    'Gear': 1,
    'Target': [
        36.1111,
        116.22286716842115],
    'LoiterPWM': [1000] * 8,
    'Mode': 'STAB',
    'Waypoint': [
        [
            36.121111111111,
            116.22211080524757
        ],
        [
            36.111200831528414,
            116.2223331950068
        ],
        [
            36.111111,
            116.22222200012719
        ]
    ],
    'WaypointID': 0,
    'WaypointType': 'Download',
    # 'Waypoint': [],
    # 'WaypointID': -1,
    # 'WaypointType': None,
    'RPM': 1600,
    'InitAltitude': -80.81,
    'IMU_State': True,
    'ACC': [
        0.1,
        0.2,
        0.3],
    'GYR': [
        0.1,
        0.2,
        0.3],
    'MAG': [
        0.1,
        0.2,
        0.3],
    'EUL': [
        0.1,
        0.2,
        0.3],
    'QUA': [
        0.1,
        0.2,
        0.3,
        0.4]}


if __name__ == '__main__':
    logger = init_logger('HEX')
    logger.debug('debug message')
    logger.info('info message')
    logger.warn('warn message')
    logger.error('error message')
    logger.critical('critical message')
