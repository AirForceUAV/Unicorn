#!/usr/bin/evn python
# coding:utf-8

import time
import math
from library import *
from mqtt_client import *
from config import config
from tools import logger


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
        if debug:
            context = {'Head2Target': 0,
                       'Epsilon': 0,
                       'State': 'STOP',
                       'pre': 'STOP',
                       'prepre': 'STOP',
                       'Distance': 100}
        else:
            self.vehicle._target = target
            CLocation = self.vehicle.get_location()
            CYaw = self.vehicle.get_heading()

            if None in (CLocation, CYaw, target):
                logger.error(
                    'GPS is None or Compass is None or target is None')
                return
            angle = angle_heading_target(CLocation, target, CYaw)
            distance = get_distance_metres(CLocation, target)
            Epsilon = math.degrees(math.asin(self.vehicle.radius / distance))

            if distance <= self.vehicle.radius:
                logger.info("Reached Target!")
                self.vehicle._target = None
                return
            context = {'Head2Target': angle,
                       'Epsilon': int(Epsilon),
                       'State': self.vehicle._state,
                       'Distance': distance,
                       'pre': self.vehicle.pre_state,
                       'prepre': self.vehicle.prepre_state}

            vehicle.condition_yaw(angle)

        message = pack(context)

        mqtt_publish(self.client, config.context_topic, message, self.userdata)

    def Guided(self):
        target = self.vehicle.get_target()
        if target is None:
            logger.warn('Target is None.Please set')
            return
        self.vehicle.publish('Mode', 'GUIDED')
        self.navigation(target)
        self.vehicle.publish('Mode', 'Loiter')
        self.vehicle.publish('Target', None)

    def RTL(self):
        target = self.vehicle.get_home()
        if target is None:
            logger.warn("HomeLocation is None.")
            return
        self.vehicle.publish('Mode', 'RTL')

        self.navigation(target)
        # self.vehicle.land()
        self.vehicle.publish('Mode', 'STAB')

    def Auto(self):
        if self.vehicle.wp.isNull():
            logger.warn('Waypoint is None.Please set')
            return
        self.vehicle.publish('Mode', 'Auto')
        watcher = CancelWatcher()
        for point in self.vehicle.wp.remain_wp():
            if watcher.IsCancel():
                self.vehicle.publish('Mode', 'Loiter')
                return
            self.navigation(point)
            if not watcher.IsCancel():
                self.vehicle.wp.add_number()

        self.vehicle.publish('Mode', 'Loiter')
        self.vehicle.wp.clear()


if __name__ == "__main__":
    from uORB import uORB
    from vehicle import Vehicle
    from library import Watcher

    ORB = uORB()
    Watcher()

    if config.has_module('Sbus'):
        # Initialize SBUS
        from sbus_receiver import Sbus_Receiver
        from sbus_sender import Sbus_Sender
        from tools import build_sbus

        com = build_sbus()
        sbus_receiver = Sbus_Receiver(ORB, com)
        sbus_receiver.start()

        while not ORB.state('Sbus'):
            time.sleep(.1)

        sbus_sender = Sbus_Sender(ORB, com)
        sbus_sender.start()

        while not ORB.state('Sender'):
            time.sleep(.1)
        print 'Sbus is OK'

    if config.has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.state('Compass'):
            time.sleep(.1)
        print 'Compass is OK'

    if config.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.state('GPS'):
            time.sleep(.1)
        print 'GPS is OK'

    if config.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.state('Baro'):
            time.sleep(.1)
        print 'Baro is OK'

    if config.has_module('IMU'):
        # Initialize IMU
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.state('IMU'):
            time.sleep(.1)
        print 'IMU is OK'

    # Save FlightLog to SD card
    # ORB.start()

    # Initialize UAV
    vehicle = Vehicle(ORB)
    lidar = Lidar(vehicle)
    # vehicle.set_target(-30, 0)
    # lidar.Guided()
    location = [36.111122, 116.222222, 10]
    lidar.navigation(location)

    while True:
        time.sleep(100)
