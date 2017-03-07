#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import math
from lidarProxy import *
from lib.science import *
from lib.tools import Singleton
from lib.config import config
from lib.logger import logger


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        self.userdata = {'vehicle': vehicle,
                         'full_id': 0,
                         'semi_id': 0,
                         'times': 0,
                         'code': False}
        self.vehicle = vehicle
        self.radius = vehicle.radius
        self.client = init_mqtt(self.userdata)
        self.client.loop_start()

    def navigation(self, target):
        self.publish('Target', target)

        context = obstacle_context(vehicle)
        if not context:
            return

        message = pack(context)
        mqtt_publish(self.client, config.context_topic, message, self.userdata)

    def Guided(self):
        target = self.vehicle.get_target()
        if target is None:
            logger.warn('Target is None.Please set')
            return

        self.publish('Mode', 'AI_GUIDED')
        self.navigation(target)

    def RTL(self):
        target = self.vehicle.get_home()
        if target is None:
            logger.warn("HomeLocation is None.")
            return

        self.vehicle.publish('Mode', 'AI_RTL')
        self.navigation(target)

    def Auto(self):
        if self.vehicle.wp.isNull():
            logger.warn('Waypoint is None.Please set')
            return
        self.vehicle.publish('Mode', 'AI_Auto')
        ID = self.vehicle.wp.ID
        points = self.vehicle.wp.points
        self.navigation(points[ID])

    def publish(self, topic, message):
        self.vehicle.publish(topic, message)

if __name__ == "__main__":
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

    # Save FlightLog to SD card
    # ORB.start()

    # Initialize UAV
    from AF_Copter.vehicle import Vehicle
    vehicle = Vehicle(ORB)
    lidar = Lidar(vehicle)

    # vehicle.download()
    vehicle.publish('Waypoint', [[36.111122, 116.222222, 10]])
    vehicle.publish('WaypointID', 0)
    lidar.Auto()
    # location = [36.111122, 116.222222, 10]
    # vehicle.publish('Target', location)
    # vehicle.set_target(-30, 0)
    # lidar.Guided()

    while True:
        time.sleep(100)
