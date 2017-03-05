#!/usr/bin/evn python
# coding:utf-8

import time

from config import config

if __name__ == '__main__':
    from uORB import uORB
    from library import Watcher

    ORB = uORB()
    Watcher()

    if config.has_module('Sbus'):
        # Initialize SBUS
        from sbus_sender import sbus_start
        sbus_start(ORB)

    if config.has_module('Compass'):
        # Initialize Compass
        from compass_module import compass_start
        compass_start(ORB)

    if config.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS_start
        GPS_start(ORB)

    if config.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro_start
        Baro_start(ORB)

    if config.has_module('IMU'):
        # Initialize IMU
        from IMU import IMU_start
        IMU_start(ORB)

    # Initialize UAV
    from vehicle import Vehicle
    vehicle = Vehicle(ORB)
    lidar = None

    if config.has_module('Lidar'):
        # Initialize Lidar
        from lidar import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    if config.has_module('Cloud'):
        # Initialize Cloud
        from cloud_module import cloud_start
        cloud_start(ORB, vehicle, lidar)

    print('completed')
