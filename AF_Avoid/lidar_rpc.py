#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import math
import paho.mqtt.client as mqtt
from lib.science import *
from lib.tools import Singleton, CancelWatcher, exe_actions
from lib.config import config
from lib.logger import logger
from oa_rpc_client import OA_Stub
import protobuf.oa_rpc_pb2 as oa
import grpc


def init_mqtt(userdata):
    client = mqtt.Client(client_id=config.client_id)
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(*config.lidar_socket)
    return client


def on_connect(client, userdata, flags, rc):
    logger.info("Connected to mqtt broker")
    topics = [(config.keyboard_topic, 2)]
    client.subscribe(topics)


def on_message(client, userdata, msg):
    # logger.debug('Received {} from Keyboard'.format(msg.payload))
    stub = userdata['stub']
    vehicle = userdata['vehicle']

    command = map(int, msg.payload.split(','))
    if oa.STOP in command:
        # print 'brake'
        client.publish('ACK', 'ack')
        vehicle.brake()
        return

    userdata['semi_id'] = userdata['semi_id'] + 1
    message = {'id': userdata['semi_id'], 'actions': command}
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


class Lidar(object):
    __metaclass__ = Singleton

    def __init__(self, vehicle):
        self.full_id = 0
        self.semi_id = 0
        self.vehicle = vehicle
        self.stub = OA_Stub()
        userdata = {'stub': self.stub,
                    'semi_id': self.semi_id,
                    'vehicle': vehicle}
        self.client = init_mqtt(userdata)
        self.client.loop_start()

    def full_auto(self):
        watcher = CancelWatcher()
        interval = 0.1
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
        logger.info('vehicle turn compelet')
        retry_times = 0
        while not watcher.IsCancel() and retry_times < 5:
            try:
                context = self.full_auto_context(self.vehicle._state)
                logger.debug('Send {} to lidar'.format(context))
                if context == True:
                    logger.info("Reached Target!")
                    self.vehicle.brake()
                    return True
            except AssertionError, e:
                logger.error(e)
                self.vehicle.brake()
                return False
            try:
                id, actions = self.stub.FullAuto(context)
                logger.debug('Receive {} from lidar'.format(actions))

            except grpc.RpcError, e:
                retry_times += 1
                logger.critical(e)
                self.vehicle.brake()
                continue

            # print 'Send {id} {current} {last_state} {last_previous_state}'.format(**context)
            # print 'Recv',actions
            try:
                assert id == self.full_id,\
                    'ID not match.Note:ExpectID:{} ReceiveID:{}'.format(
                        self.full_id, id)
                # actions= [0xee]
                exe_actions(self.vehicle, actions)
                self.vehicle._state = actions

            except AssertionError, e:
                retry_times += 1
                logger.error(e)
                logger.warn('Retry times:{}'.format(retry_times))
                continue
            retry_times = 0
            time.sleep(interval)
        return False

    def Guided(self):
        logger.debug('Guided(AI) start ...')
        flag = True
        try:
            target = self.vehicle.get_target()
        except AssertionError, e:
            logger.error(e)
            return False
        self.publish('Mode', 'AI_GUIDED')
        result = self.full_auto()
        if not result:
            logger.error("Navigation except exit")
            flag = False
        self.vehicle._finally()
        return flag

    def RTL(self):
        logger.debug('RTL(AI) start ...')
        flag = True
        try:
            home = self.vehicle.get_home()
        except AssertionError, e:
            logger.error(e)
            return False
        self.vehicle.publish('Target', home)
        self.vehicle.publish('Mode', 'AI_RTL')
        result = self.full_auto()
        if not result:
            logger.error("Navigation except exit")
            flag = False
        self.vehicle._finally()
        return flag

    def Auto(self):
        logger.debug('Auto(AI) start ...')
        flag = True
        if self.vehicle.wp.isNull():
            logger.warn('Waypoint is Null.Please set Waypoint')
            return False
        self.publish('Mode', 'AI_Auto')
        watcher = CancelWatcher()
        for point in self.vehicle.wp.points:
            if watcher.IsCancel():
                logger.warn('Cancel Auto')
                flag = False
                break
            self.publish('Target', point)
            result = self.full_auto()
            if not result:
                logger.error("Navigation except exit")
                flag = False
                break
            self.wp.add_number()

        self.vehicle.Auto_finally()
        return flag

    def full_auto_context(self, command):
        def context_release():
            radius = self.vehicle.radius
            target = self.vehicle.get_target()
            CLocation = self.vehicle.get_location()
            CYaw = self.vehicle.get_heading()

            angle = angle_heading_target(CLocation, target, CYaw)
            distance = get_distance_metres(CLocation, target)

            if distance <= radius:
                return True

            Epsilon = math.degrees(math.asin(radius / distance))
            Epsilon = self.vehicle.Filter_Epsilon(Epsilon)
            # if not vehicle.InAngle(angle, 90) or distance <= radius:

            if command != self.vehicle._state:
                self.vehicle.prepre_state = self.vehicle.pre_state
                self.vehicle.pre_state = self.vehicle._state
                self.vehicle._state = command
                # print 'brake'
                self.vehicle.brake()

            context = {
                'id': self.full_id,
                'target': angle,
                'epsilon': Epsilon,
                'current': self.vehicle._state,
                'last_state': self.vehicle.pre_state,
                'last_previous_state': self.vehicle.prepre_state}
            # print context
            return context

        self.full_id += 1
        context = context_release()
        return context

    def semi_auto(self, command):
        if command not in [oa.LEFT_ROLL, oa.RIGHT_ROLL, oa.FORWARD, oa.BACKWARD]:
            logger.error('command:{} is invalid'.format(command))
            return

        self.semi_id = self.semi_id + 1
        message = {'id': self.semi_id, 'actions': [command]}
        logger.debug('Send {} to Lidar'.format(message))
        try:
            id, actions = self.stub.SemiAuto(message)
            logger.debug(
                'Received id:{} actions:{} from Lidar'.format(id, actions))
            if id != self.semi_id:
                logger.error(
                    'ID not match.Note:ExpectID:{} ReceiveID:{}'.format(self.semi_id, id))
                return
            exe_actions(self.vehicle, actions)
        except grpc.RpcError, e:
            logger.critical(e)
        except AssertionError, e:
            logger.error(e)

    def publish(self, topic, message):
        self.vehicle.publish(topic, message)


if __name__ == "__main__":
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    from AF_Copter.vehicle import init_vehicle
    vehicle = init_vehicle(ORB)
    lidar = Lidar(vehicle)
    time.sleep(1)
    vehicle.set_target(-30, 0)
    lidar.Guided()
    # lidar.RTL()
    # lidar.Auto()
    # lidar.semi_auto(16)
    # print 'Done'
    while True:
        time.sleep(100)
