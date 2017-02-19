#!/usr/bin/evn python
# coding:utf-8

import time
import math
from library import CancelWatcher, get_distance_metres
from library import angle_heading_target
from library import Singleton
from tools import exe_cmd
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import paho.mqtt.subscribe as subscribe
from debug_env import env

full_auto_topic = "Full-Automatic"
semi_auto_topic = "Semi-Automatic"
context_topic = 'Context'
control_topic = 'Control'
# host = '192.168.31.10'
host = 'localhost'
port = 12345
full_client_id = 'FULLFC'
semi_client_id = 'SEMIFC'


def obstacle_context(vehicle):

    global env

    def context_release():
        target = vehicle._target
        CLocation = vehicle.get_location()
        CYaw = vehicle.get_heading()

        if CLocation is None or CYaw is None or target is None:
            vehicle._error('GPS is None or Compass is None or target is None')
            return None
        angle = angle_heading_target(CLocation, target, CYaw)
        distance = get_distance_metres(CLocation, target)
        Epsilon = math.degrees(math.asin(vehicle.radius / distance))
        state = vehicle._state
        context = {'Head2Target': angle,
                   'Epsilon': int(Epsilon),
                   'State': state,
                   'Distance': distance}
        return context

    def context_debug():
        context = {'Head2Target': 0,
                   'Epsilon': 0,
                   'State': vehicle._state,
                   'Distance': 100}
        return context
    if env == 'debug':
        return context_debug()
    else:
        return context_release()


def on_connect(client, vehicle, rc):
    global full_auto_topic, semi_auto_topic
    vehicle._debug("Connected cloud with result code {}".format(rc))
    # client.subscribe([(full_auto_topic, 2), (semi_auto_topic, 2)])
    client.subscribe(full_auto_topic, qos=2)


def on_message(client, vehicle, msg):
    global full_auto_topic, semi_auto_topic, context_topic
    topic = msg.topic
    command = msg.payload.strip()
    isValid = exe_cmd(vehicle, command)
    if not isValid:
        return

    if topic == full_auto_topic:
        context = obstacle_context(vehicle)
        if context is None:
            return
        distance = context['Distance']
        angle = context['Head2Target']
        # if not vehicle.InAngle(angle, 90) or distance <= vehicle.radius:
        if distance <= vehicle.radius:
            vehicle._log("Reached Target!")
            vehicle.brake()
            vehicle.pre_state = vehicle.prepre_state = vehicle._state = 'STOP'
            return

        if command != vehicle._state:
            vehicle.prepre_state = vehicle.pre_state
            vehicle.pre_state = vehicle._state
            vehicle._state = command
            context['State'] = command
            vehicle.brake()

        context['pre'] = vehicle.pre_state
        context['prepre'] = vehicle.prepre_state

        message = '{Head2Target} {Epsilon} {State} {pre} {prepre}'.format(
            **context)

        # message = raw_input('Next')
        time.sleep(3)
        print message
        client.publish(context_topic, message, qos=2)
        vehicle._debug('{Distance} {Head2Target} {Epsilon}'.format(**context))
    # elif topic == semi_auto_topic:
    #     time.sleep(1)
    #     vehicle.brake()


def movement(vehicle, command):
    global control_topic, semi_auto_topic, semi_client_id, host, port
    other_param = {'client_id': semi_client_id,
                   'hostname': host, 'port': port, 'qos': 2}
    print 'Send:', command
    publish.single(control_topic, command, **other_param)
    msg = subscribe.simple(semi_auto_topic, **other_param)
    cmd = msg.payload.strip()
    exe_cmd(vehicle, cmd)


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        global host, port, full_client_id
        self.client = mqtt.Client(client_id=full_client_id)
        self.client.user_data_set(vehicle)
        self.client.on_connect = on_connect  # callback when connected
        self.client.on_message = on_message  # callback when received message
        self.client.connect(host, port)
        self.client.loop_start()
        self.vehicle = vehicle
        self.radius = vehicle.radius

    def navigation(self, target):
        global context_topic
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
        print message
        # self.vehicle._debug('Context:{}'.format(message))
        self.client.publish(context_topic, message, qos=2)

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

    def keycontrol(self):
        import keyboard

        info = {'up': 'FORWARD', 'down': 'BACKWARD',
                'left': 'LEFT_ROLL', 'right': 'RIGHT_ROLL',
                'space': 'STOP', 'esc': 'esc', 'page up': 'UP',
                'page down': 'DOWN'}

        def callback(event):
            eventType = event.event_type
            name = event.name
            if eventType == 'down' and name in info:
                return True
            else:
                return False

        while True:
            event = keyboard.read_key(callback)
            name = event.name
            # print name
            if name == 'esc':
                print 'esc'
                return
            movement(self.vehicle, info[name])

        # keyboard.wait('esc')

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
    # vehicle.set_target(-30, 0)
    # lidar.Guided()
    # lidar.keycontrol()
    location = [36.111122, 116.222222, 10]
    lidar.navigation(location)

    while True:
        time.sleep(100)
