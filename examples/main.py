#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
from lib.config import config


if __name__ == '__main__':
    from AF_uORB.uORB import uORB
    from lib.tools import Watcher

    ORB = uORB()
    Watcher()

    if config.has_module('Sbus'):
        # Initialize SBUS
        from AF_Sbus.sender import sbus_start
        sbus_start(ORB)

    if config.has_module('Compass'):
        # Initialize Compass
        from AF_Sensors.compass import compass_start
        compass_start(ORB)

    if config.has_module('GPS'):
        # Initialize GPS
        from AF_Sensors.GPS import GPS_start
        GPS_start(ORB)

    if config.has_module('Baro'):
        # Initialize Barometre
        from AF_Sensors.Baro import Baro_start
        Baro_start(ORB)

    if config.has_module('IMU'):
        # Initialize IMU
        from AF_Sensors.IMU import IMU_start
        IMU_start(ORB)

    # Initialize UAV
    from AF_Copter.vehicle import Vehicle
    vehicle = Vehicle(ORB)
    lidar = None

    if config.has_module('Lidar'):
        # Initialize Lidar
        from AF_Avoid.lidar import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    if config.has_module('GCS'):
        # Initialize GCSProxy
        from AF_GCS.GCSProxy import GCS_start
        cloud_start(ORB, vehicle, lidar)

    logger.info('completed')
