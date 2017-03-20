#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import math
import time
import paho.mqtt.client as mqtt
from AF_Keyboard.KB_Control import exe_cmd
from lib.science import *
from lib.logger import logger
from lib.tools import *
from lib.config import config


def init_mqtt(userdata):
    client = mqtt.Client(client_id=config.client_id)
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(*config.lidar_socket)
    return client


def on_connect(client, userdata, flags, rc):
    logger.info("Connected to mqtt broker")
    topics = [(config.full_auto_topic, 2),
              (config.semi_auto_topic, 2),
              (config.keyboard_topic, 2)]
    client.subscribe(topics)


def on_message(client, userdata, msg):
    message = msg.payload
    topic = msg.topic
    watcher = CancelWatcher()
    if watcher.IsCancel():
        # client.unsubscribe(topic)
        Finally(vehicle)
        return

    vehicle = userdata['vehicle']

    logger.debug('From topic [{}] receive ({})'.format(topic, message))

    if topic == config.keyboard_topic:
        if message == 'esc':
            print 'Exit semi_auto'
            # client.unsubscribe(keyboard_topic)
            Finally(vehicle)
            return
        mqtt_publish(client, config.control_topic, message, userdata)
        # mqtt_publish(client, control_topic, message, userdata, True)
    else:
        revid, command = unpack(message)
        result = revid_is_valid(topic, revid, userdata)
        if result:
            exe_cmd(vehicle, command)

        if topic == config.full_auto_topic:
            full_publish(client, userdata, command)


def revid_is_valid(topic, revid, userdata):
    topic2key = {config.full_auto_topic: 'full_id',
                 config.semi_auto_topic: 'semi_id'}

    key = topic2key[topic]
    expectid = userdata[key]
    if revid == expectid:
        userdata['code'] = True
        return True
    else:
        logger.error(
            'Revid is unvalid revid:{} expectid:{}'.format(revid, expectid))
        return False


def full_publish(client, userdata, command):
    interval = 3
    vehicle = userdata['vehicle']
    context = obstacle_context(vehicle, command)
    # print 'context', context
    if not context:
        Finally(vehicle)
        return
    elif context == True:
        mode = vehicle.subscribe('Mode')
        if mode == 'AI_Auto':
            ID = vehicle.wp.ID
            points = vehicle.wp.points
            wp_count = len(points)
            if ID == wp_count - 1:
                logger.info('{} have been finished'.format(mode))
                Finally(vehicle)
                return
            else:
                vehicle.wp.add_number()
                vehicle.publish('Target', points[ID + 1])
                context = obstacle_context(vehicle)
                if not context:
                    Finally(vehicle)
                    return
        else:
            logger.info('{} have been finished'.format(mode))
            Finally(vehicle)
            return
    message = pack(context)
    # raw_input('Next')
    time.sleep(interval)
    # client.publish(config.context_topic, message, qos=2)
    mqtt_publish(client, config.context_topic, message, userdata)
    logger.debug('{Distance} {Head2Target} {Epsilon}'.format(**context))


def pack(context):
    message = '{Head2Target} {Epsilon} {State} {pre} {prepre}'.format(
        **context)
    return message


def unpack(message):
    '''id:int command:"FORWARD LEFT_ROLL") '''
    msg_ls = message.split(' ')
    if len(msg_ls) < 2:
        return -1, None
    try:
        mid = int(msg_ls[0])
        command = ' '.join(msg_ls[1:])
    except ValueError as e:
        mid = -1
        command = None
    finally:
        return mid, command


def obstacle_context(vehicle, command=None):
    def context_debug():
        context = {'Head2Target': 0,
                   'Epsilon': 0,
                   'State': 'STOP',
                   'pre': 'STOP',
                   'prepre': 'STOP',
                   'Distance': 100}
        return context

    def context_release():
        target = vehicle.subscribe('Target')
        CLocation = vehicle.get_location()
        CYaw = vehicle.get_heading()
        if target is None:
            logger.error('Target is None')
            return False
        elif CYaw is None:
            return False
        elif CLocation is None:
            return False

        angle = angle_heading_target(CLocation, target, CYaw)
        distance = get_distance_metres(CLocation, target)
        Epsilon = math.degrees(math.asin(vehicle.radius / distance))

        # if not vehicle.InAngle(angle, 90) or distance <= vehicle.radius:
        if distance <= vehicle.radius and command is not None:
            logger.info("Reached Target!")
            return True

        if command == None:
            vehicle.condition_yaw(angle)
        elif command != vehicle._state:
            vehicle.prepre_state = vehicle.pre_state
            vehicle.pre_state = vehicle._state
            vehicle._state = command
            vehicle.brake()

        context = {'Head2Target': angle,
                   'Epsilon': int(Epsilon),
                   'State': vehicle._state,
                   'Distance': distance,
                   'pre': vehicle.pre_state,
                   'prepre': vehicle.prepre_state}
        return context

    context = context_debug() if config.debug else context_release()
    return context


# def mqtt_publish2(client, topic, message, userdata, blocking=False):
#     topic2id = {context_topic: 'full_id', control_topic: 'semi_id'}
#     retry = 5
#     interval = 1
#     key_mid = topic2id[topic]
#     for times in range(retry):
#         userdata[key_mid] = userdata[key_mid] + 1
#         data = str(userdata[key_mid]) + " " + message
#         print('Send message:({})'.format(data))
#         infot = client.publish(topic, data, qos=2)
#         if blocking:
#             infot.wait_for_publish()
#         for i in range(interval * 10):
#             if userdata['code']:
#                 userdata['code'] = False
#                 return
#             time.sleep(.1)
#         print('Retry No.%d times' % (times + 1))
#     userdata['code'] = False
#     print('Mqtt broker has no reponses')


def mqtt_publish(client, topic, message, userdata, blocking=False):
    topic2id = {config.context_topic: 'full_id',
                config.control_topic: 'semi_id'}
    key_mid = topic2id[topic]
    userdata[key_mid] = userdata[key_mid] + 1
    data = str(userdata[key_mid]) + " " + message
    logger.debug('Send message:({}) to topic:[{}]'.format(data, topic))
    infot = client.publish(topic, data, qos=2)

    if blocking:
        infot.wait_for_publish()


def Finally(vehicle):
    vehicle.publish('Target', None)
    vehicle.pre_state = vehicle.prepre_state = vehicle._state = 'STOP'
    if vehicle.subscribe('Mode') == 'AI_Auto':
        vehicle.wp.clear()
    vehicle.publish('Mode', 'Loiter')
    vehicle.brake()

if __name__ == '__main__':
    userdata = {'vehicle': None,
                'full_id': 0,
                'semi_id': 0,
                'times': 0,
                'code': False}
    client = init_mqtt(userdata)
    client.loop_start()

    while True:
        mqtt_publish(client, config.control_topic, 'STOP', userdata, True)
