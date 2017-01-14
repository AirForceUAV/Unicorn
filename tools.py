#!/usr/bin/evn python
# coding:utf-8

import serial
import sys
import time

open_module = [
    'Sbus',
    # 'Compass',
    # 'GPS',
    # 'Baro',
    # 'IMU',
    # 'Lidar',
    'Cloud',
]

commands = [
    # 'arm()',
    # 'set_channels_mid()',
    # 'set_gear(2)',
    # 'yaw_left_brake()',
    # 'yaw_right_brake()',
    # 'roll_left_brake()',
    # 'roll_right_brake()',
    # 'forward_brake()',
    # 'backward_brake()',
    # 'up_brake()',
    # 'down_brake()',
    'condition_yaw(30)',
    'condition_yaw(300)',
    # 'set_target(-20, 0)',
    # 'Guided()',
    # 'download()',
    # 'Auto()',
    'disarm()',
]


def build_sbus():
    print 'Initialize Radio ...'
    portname = '/dev/sbus'
    # portname = '/dev/ttyAMA0'
    print 'Connecting to', portname
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
            print "{0}:{1}".format(*info)
            time.sleep(.5)

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
    'Mode': 'Loiter',
    'Waypoint': [],
    'WaypointID': -1,
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
