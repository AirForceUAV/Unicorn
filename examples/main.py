#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
from lib.config import config
from lib.logger import logger


if __name__ == '__main__':
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    # Initialize UAV
    from AF_Copter.vehicle import init_vehicle
    vehicle = init_vehicle(ORB)
    lidar = None

    if config.has_module('Lidar'):
        # Initialize Lidar
        from AF_Avoid.lidar_rpc import Lidar
        lidar = Lidar(vehicle)

    # Save FlightLog to SD
    # ORB.start()

    if config.has_module('GCS'):
        # Initialize GCSProxy
        from AF_GCS.GCSProxy import GCS_start
        GCS_start(ORB, vehicle, lidar)
        
    while True:
        time.sleep(100000)

    logger.info('completed')
