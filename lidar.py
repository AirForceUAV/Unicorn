#!/usr/bin/evn python
# coding:utf-8

import os
import time
import math
from library import CancelWatcher, get_distance_metres
from library import angle_heading_target
from library import Singleton
import paho.mqtt.client as mqtt


def obstacle_context(vehicle):
    target = vehicle._target
    CLocation = vehicle.get_location()
    CYaw = vehicle.get_heading()

    if CLocation is None or CYaw is None or target is None:
        vehicle._error('GPS is None or Compass is None or target is None')
        return None
    angle = angle_heading_target(CLocation, target, CYaw)
    Epsilon = math.degrees(math.asin(vehicle.radius / distance))
    state = vehicle.state
    distance = get_distance_metres(CLocation, target)
    context = {'Head2Target': angle,
               'Epsilon': int(Epsilon),
               'State': state,
               'Distance': distance}
    return context


def _obstacle_context(vehicle):
    context = {'Head2Target': 0,
               'Epsilon': 0,
               'State': vehicle.state,
               'Distance': 100}
    return context


def on_connect(client, vehicle, rc):
    topic = "ObstacleSystem"
    # topic = "$SYS/#"
    vehicle._debug("Connected cloud with result code {}".format(rc))
    # Subscribe Topic "Command"
    client.subscribe(topic, qos=2)


def on_message(client, vehicle, msg):
    input = {'STOP': {}, 'FORWARD': {'ELE': 1},
             'LEFT_YAW': {'RUD': -1}, 'RIGHT_YAW': {'RUD': 1},
             'LEFT_ROLL': {'AIL': -1}, 'RIGHT_ROLL': {'AIL': 1}}

    context = obstacle_context(vehicle)
    if context is None:
        return
    distance = context['Distance']
    angle = context['Head2Target']
    if not vehicle.InAngle(angle, 90) or distance <= vehicle.radius:
        vehicle._log("Reached Target!")
        vehicle.brake()
        vehicle.pre_state = vehicle.prepre_state = vehicle.state = 'STOP'
        return

    command = msg.payload
    vehicle._debug('command:{}'.format(command))
    if command not in input:
        vehicle._error('Command({}) is unvalid!'.format(command))
        return
    if command is not vehicle.state:
        vehicle.prepre_state = vehicle.pre_state
        vehicle.pre_state = vehicle.state
        vehicle.state = command
        context['State'] = command
        # vehicle.brake()
    action = input[command]
    vehicle.control_FRU(**action)
    context['pre'] = vehicle.pre_state
    context['prepre'] = vehicle.prepre_state

    message = '{Head2Target} {Epsilon} {State} {pre} {prepre}'.format(
        **context)
    # vehicle._debug('Context:{}'.format(message))
    # message = raw_input('Next')
    time.sleep(2)
    # print message
    client.publish('Context', message, qos=2)


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        # IP = '192.168.31.10'
        IP = '192.168.31.5'
        # IP = '127.0.0.1'
        port = 12345
        # IP = "iot.eclipse.org"
        # port = 1883
        self.client = mqtt.Client(client_id='FC')
        self.client.user_data_set(vehicle)
        self.client.on_connect = on_connect
        self.client.on_message = on_message
        self.client.connect(IP, port)
        self.client.loop_start()
        self.vehicle = vehicle
        self.radius = vehicle.radius

    def navigation(self, target):
        if target is None:
            self.vehicle._warning('Target is None')
            return
        self.vehicle._target = target
        context = obstacle_context(self.vehicle)
        if context is None:
            return
        init_angle = context['Head2Target']
        self.vehicle.condition_yaw(init_angle)
        context['pre'] = vehicle.pre_state
        context['prepre'] = vehicle.prepre_state
        message = '{Head2Target} {Epsilon} {State} {pre} {prepre}'.format(
            **context)
        # self.vehicle._debug('Context:{}'.format(message))
        self.client.publish('Context', message, qos=2)

    def Guided(self):
        target = self.vehicle.get_target()
        if target is None:
            self.vehicle._warning('Target is None')
            return
        self.vehicle.publish('Mode', 'GUIDED')

        self.navigation(target)
        self.vehicle.publish('Mode', 'Loiter')
        self.vehicle.publish('Target', None)

    def RTL(self):
        target = self.vehicle.get_home()
        if target is None:
            self.vehicle._warning("HomeLocation is None!")
            return
        self.vehicle.publish('Mode', 'RTL')

        self.navigation(target)
        # self.vehicle.land()
        self.vehicle.publish('Mode', 'STAB')

    def Auto(self):
        if self.vehicle.wp.isNull():
            self.vehicle._warning('Waypoint is None')
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

    if ORB.has_module('Sbus'):
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

    if ORB.has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.state('Compass'):
            time.sleep(.1)
        print 'Compass is OK'

    if ORB.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.state('GPS'):
            time.sleep(.1)
        print 'GPS is OK'

    if ORB.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.state('Baro'):
            time.sleep(.1)
        print 'Baro is OK'

    if ORB.has_module('IMU'):
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
    vehicle.set_target(-30, 0)
    lidar.Guided()
    # location = [36.111122, 116.222222, 10]
    # lidar.navigation(location)
    while True:
        time.sleep(100)
