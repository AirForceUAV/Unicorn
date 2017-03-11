#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import math
import paho.mqtt.client as mqtt
from lib.science import *
from lib.tools import Singleton, CancelWatcher
from lib.config import config
from lib.logger import logger
from oa_rpc_client import OA_Stub
import oa_rpc_pb2
import grpc


def init_mqtt(userdata):
    client = mqtt.Client(client_id=config.client_id)
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(*config.mqtt_socket)
    return client


def on_connect(client, userdata, flags, rc):
    logger.info("Connected to mqtt broker")
    topics = [(config.keyboard_topic, 2)]
    client.subscribe(topics)


def on_message(client, userdata, msg):
    # logger.debug('Received {} from Keyboard'.format(msg.payload))
    stub = userdata['stub']
    vehicle = userdata['vehicle']
    userdata['semi_id'] = userdata['semi_id'] + 1
    command = msg.payload
    message = {'id': userdata['semi_id'], 'actions': [int(command)]}
    logger.debug('Send {} to Lidar'.format(message))
    try:
        id, actions = stub.SemiAuto(message)
        logger.debug(
            'Received id:{} actions:{} from Lidar'.format(id, actions))
        if id != userdata['semi_id']:
            logger.error('ID not match.Note:ExceptID:{} ReceiveID:{}'.format(
                userdata['semi_id'], id))
            return
        exe_actions(vehicle, actions)
    except grpc.RpcError, e:
        logger.critical(e)
    except AssertionError, e:
        logger.error(e)
    finally:
        # time.sleep(1)
        client.publish('ACK', 'ack')


def exe_actions(vehicle, RawActions):
    map_action = {
        '0': {'Mode': 2},
        '1': {'ELE': 1}, '2': {'ELE': -1},
        '4': {'RUD': 1}, '8': {'RUD': -1},
        '16': {'AIL': 1}, '32': {'AIL': -1},
        '64': {'THR': 1}, '128': {'THR': -1},
        '0xee': None, '0xff': None
    }
    actions = {}
    for action in RawActions:
        dictaction = map_action.get(str(action))
        assert not (dictaction is None or dictaction.keys()[0] in actions),\
            'Command is invalid. Note: Command is {}'.format(Raw)
        actions = dict(actions, **dictaction)

    logger.debug('Execute Action:{}'.format(actions))
    vehicle.control_FRU(**actions)


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        self.full_id = 0
        self.semi_id = 0
        self.vehicle = vehicle
        self.stub = OA_Stub()
        userdata = {'stub': self.stub, 'semi_id': 0, 'vehicle': self.vehicle}
        self.client = init_mqtt(userdata)
        self.client.loop_start()

    def navigation(self):
        watcher = CancelWatcher()
        interval = 2
        radius = self.vehicle.radius
        try:
            target = self.vehicle.get_target()
            CLocation = self.vehicle.get_location()
            CYaw = self.vehicle.get_heading()
            angle = angle_heading_target(CLocation, target, CYaw)
            self.vehicle.condition_yaw(angle)
        except AssertionError, e:
            logger.error(e)
            self.vehicle.brake()
            return False
        retry_times = 0
        while not watcher.IsCancel() and retry_times < 5:
            try:
                context = self.full_auto_context(self.vehicle._state)
                if context == True:
                    return True
            except AssertionError, e:
                logger.error(e)
                self.vehicle.brake()
                return False
            try:
                id, actions = self.stub.FullAuto(context)
            except grpc.RpcError, e:
                logger.critical(e)
                self.vehicle.brake()
                return False

            # print 'Send {id} {current} {last_state} {last_previous_state}'.format(**context)
            # print 'Recv',actions
            try:
                assert id == self.full_id,\
                    'ID not match.Note:ExceptID:{} ReceiveID:{}'.format(
                        self.full_id, id)
                # actions= [0xee]
                exe_actions(self.vehicle, actions)
                retry_times = 0
            except AssertionError, e:
                retry_times += 1
                logger.error(e)
                logger.warn('Retry times:{}'.format(retry_times))
                continue
            time.sleep(interval)
        return False

    def Guided(self):
        logger.debug('Guided(AI) start ...')
        try:
            target = self.vehicle.get_target()
        except AssertionError, e:
            logger.error(e)
            return
        self.publish('Mode', 'AI_GUIDED')
        self.navigation()
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

    def RTL(self):
        logger.debug('RTL(AI) start ...')
        try:
            target = self.vehicle.get_home()
        except AssertionError, e:
            logger.error(e)
            return
        self.vehicle.publish('Target', target)
        self.vehicle.publish('Mode', 'AI_RTL')
        self.navigation()
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

    def Auto(self):
        logger.debug('Auto(AI) start ...')
        if self.vehicle.wp.isNull():
            logger.warn('Waypoint is Null.Please set Waypoint')
            return
        self.publish('Mode', 'AI_Auto')
        watcher = CancelWatcher()
        for point in self.vehicle.wp.points:
            if watcher.IsCancel():
                break
            self.publish('Target', point)
            result = self.navigation()
            if not result:
                break
            self.wp.add_number()

        self.publish('Mode', 'Loiter')
        self.publish('Target', None)
        self.wp.clear()

    def full_auto_context(self, command):
        def context_release():
            radius = self.vehicle.radius
            target = self.vehicle.get_target()
            CLocation = self.vehicle.get_location()
            CYaw = self.vehicle.get_heading()

            angle = angle_heading_target(CLocation, target, CYaw)
            distance = get_distance_metres(CLocation, target)
            Epsilon = math.degrees(math.asin(radius / distance))

            # if not vehicle.InAngle(angle, 90) or distance <= radius:
            if distance <= radius:
                logger.info("Reached Target!")
                return True

            if command != self.vehicle._state:
                self.vehicle.prepre_state = vehicle.pre_state
                self.vehicle.pre_state = vehicle._state
                self.vehicle._state = command
                # print 'brake'
                self.vehicle.brake()

            context = {
                'id': self.full_id,
                'target': angle,
                'epsilon': int(Epsilon),
                'current': self.vehicle._state,
                'last_state': self.vehicle.pre_state,
                'last_previous_state': self.vehicle.prepre_state}
            return context

        self.full_id += 1
        context = context_release()
        return context

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

    # vehicle.publish('Target', [36, 117])
    vehicle.set_target(-20,0)
    lidar.Guided()
    # lidar.RTL()
    # lidar.Auto()
    # print 'Done'
    while True:
        time.sleep(100)
