#!/usr/bin/evn python
# coding:utf-8

import os
import time
import math
from library import CancelWatcher, get_distance_metres
from library import angle_heading_target
from library import Singleton
import paho.mqtt.client as mqtt
from tools import _log


def obstacle_context(vehicle, target):
    CLocation = vehicle.get_location()
    CYaw = vehicle.get_heading()

    if CLocation is None or CYaw is None or target is None:
        _log('ERROR:GPS is None or Compass is None or target is None')
        return None
    angle = angle_heading_target(CLocation, target, CYaw)
    Epsilon = math.degrees(math.asin(vehicle.radius / distance))
    state = vehicle.state
    distance = get_distance_metres(CLocation, vehicle.target)
    context = {'Head2Target': angle,
               'Epsilon': int(Epsilon),
               'State': state,
               'Distance': distance}
    return context


def on_connect(client, userdata, rc):
    topic = "ObstacleSystem"
    # topic = "$SYS/#"
    print "Connected cloud with result code " + str(rc)
    # Subscribe Topic "Command"
    client.subscribe(topic, qos=2)


def on_message(client, vehicle, msg):
    input = {'STOP': {}, 'FORWARD': {'ELE': 1},
             'LEFT_YAW': {'RUD': -1}, 'RIGHT_YAW': {'RUD': 1},
             'LEFT_ROLL': {'AIL': -1}, 'RIGHT_ROLL': {'AIL': 1}}
    context = obstacle_context(vehicle)
    distance = context['Distance']
    angle = context['Head2Target']
    if not (vehicle.InAngle(angle, 90) and distance > vehicle.radius):
        print "Reached Target!"
        vehicle.brake()
        return
    command = msg.payload
    print 'command:', command
    if command not in input:
        print('Error:Command is unvalid!')
        return
    vehicle.state = command
    action = input[command]
    vehicle.control_FRU(**action)

    message = '{Head2Target} {Epsilon} {State}'.format(**context)

    client.publish('context', message, qos=0)


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        IP = '192.168.31.10'
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
        context = obstacle_context(self.vehicle, target)
        if context is None:
            return
        init_angle = context['Head2Target']
        self.vehicle.condition_yaw(init_angle)
        message = '{Head2Target} {Epsilon} {State}'.format(**context)
        self.client.publish('context', message, qos=2)

    def Guided(self):
        target = self.vehicle.get_target()
        if target is None:
            _log('Target is None')
            return
        self.vehicle.publish('Mode', 'GUIDED')

        self.navigation(target)
        self.vehicle.publish('Mode', 'Loiter')
        self.vehicle.publish('Target', None)

    def RTL(self):
        target = self.vehicle.get_home()
        if target is None:
            _log("HomeLocation is None!")
            return
        self.vehicle.publish('Mode', 'RTL')

        self.navigation(target)
        # self.vehicle.land()
        self.vehicle.publish('Mode', 'STAB')

    def Auto(self):
        if self.vehicle.wp.isNull():
            print 'Waypoint is None'
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

    Watcher()
    ORB = uORB()
    vehicle = Vehicle(ORB)
    lidar = Lidar(vehicle)
    location = [36.111122, 116.222222, 10]
    lidar.navigation(location)
    time.sleep(10000)
