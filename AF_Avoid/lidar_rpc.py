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
    if oa_rpc_pb2.STOP in command:
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


def unpack_actions(actions):
    map_action = {
        oa_rpc_pb2.STOP: {},
        oa_rpc_pb2.FORWARD: {'ELE': 1}, oa_rpc_pb2.BACKWARD: {'ELE': -1},
        oa_rpc_pb2.RIGHT_YAW: {'RUD': 1}, oa_rpc_pb2.LEFT_YAW: {'RUD': -1},
        oa_rpc_pb2.RIGHT_ROLL: {'AIL': 1}, oa_rpc_pb2.LEFT_ROLL: {'AIL': -1},
        oa_rpc_pb2.UP: {'THR': 1}, oa_rpc_pb2.DOWN: {'THR': -1},
        oa_rpc_pb2.INVALID: None, oa_rpc_pb2.ANY: {}
    }
    result = {}
    for action in actions:
        if action in [oa_rpc_pb2.STOP, oa_rpc_pb2.ANY]:
            result = {}
            break
        dictaction = map_action.get(action)
        assert not (dictaction is None or dictaction.keys()[0] in result),
            'Command is invalid. Note: Command is {}'.format(actions)
        result = dict(result, **dictaction)
    return result


def exe_actions(vehicle, actions):
    result = unpack_actions(actions)

    logger.debug('Execute Action:{}'.format(result))
    vehicle.control_FRU(**result)


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
        logger.info('vehicle turn compelet')
        retry_times = 0
        while not watcher.IsCancel() and retry_times < 5:
            try:
                context = self.full_auto_context(self.vehicle._state)
                if context == True:
                    logger.info("Reached Target!")
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
        self._end()
        # lidar.Guided() None)

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
        self._end()

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

        self._end()
        self.wp.clear()

    def _end(self):
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

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
            print context
            return context

        self.full_id += 1
        context = context_release()
        return context

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

    # vehicle.publish('Target', [36, 117])
    # vehicle.set_target(-20, 0)
    # lidar.Guided()
    # lidar.RTL()
    # lidar.Auto()
    # print 'Done'
    while True:
        time.sleep(100)
