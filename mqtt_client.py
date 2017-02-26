#!/usr/bin/evn python
# coding:utf-8

import math
import time
import paho.mqtt.client as mqtt
from keyboard_control import keyboard_event_wait, exe_cmd
from tools import logger
from library import *
from config import *


def print_userdata(userdata):
    print "{pub_id} {code}".format(**userdata)


def init_mqtt(userdata):
    client = mqtt.Client(client_id=client_id)
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(*mqtt_socket)
    return client


def on_connect(client, userdata, flags, rc):
    logger.info("Connected to mqtt broker")
    client.subscribe([(full_auto_topic, 2), (semi_auto_topic, 2)])


def on_message(client, userdata, msg):
    wathcer = CancelWatcher()
    topic2id = {full_auto_topic: 'full_id', semi_auto_topic: 'semi_id'}
    message = msg.payload
    topic = msg.topic

    revid, command = unpack(message)
    logger.debug('Recv message:({})'.format(message))
    vehicle = userdata['vehicle']
    key_mid = topic2id[topic]

    if revid == userdata[key_mid]:
        userdata['code'] = True
        exe_cmd(vehicle, command)
    else:
        logger.error('Revid is unvalid revid:{} mid:{}'.format(
            revid, userdata[key_mid]))
        userdata['times'] = userdata['times'] + 1
        if userdata['times'] >= 10:
            print('exit')
            userdata['times'] = 0
            return

    if topic == full_auto_topic and not wathcer.IsCancel():
        full_publish(client, userdata, command)
    elif topic == semi_auto_topic and not wathcer.IsCancel():
        semi_publish(client, userdata)


def full_publish(client, userdata, command):
    global context_topic
    interval = 3
    vehicle = userdata['vehicle']
    context = obstacle_context(vehicle, command)
    if context is None:
        return
    message = pack(context)
    # raw_input('Next')
    time.sleep(interval)
    # client.publish(context_topic, message, qos=2)
    mqtt_RTO(client, context_topic, message, userdata)
    logger.debug('{Distance} {Head2Target} {Epsilon}'.format(**context))


def semi_publish(client, userdata):
    global control_topic
    command = keyboard_event_wait()
    print 'command', command
    if command == 'esc':
        logger.info('esc')
        return True
    # infot = client.publish(control_topic, command, qos=2)
    # infot.wait_for_publish()
    mqtt_RTO(client, control_topic, command, userdata)
    return False


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
    def context_alpha():
        target = vehicle._target
        CLocation = vehicle.get_location()
        CYaw = vehicle.get_heading()

        if None in (CLocation, CYaw, target):
            logger.error('GPS is None or Compass is None or target is None')
            return None
        angle = angle_heading_target(CLocation, target, CYaw)
        distance = get_distance_metres(CLocation, target)
        Epsilon = math.degrees(math.asin(vehicle.radius / distance))

        # if not vehicle.InAngle(angle, 90) or distance <= vehicle.radius:
        if distance <= vehicle.radius:
            logger.info("Reached Target!")
            vehicle._target = None
            vehicle.pre_state = vehicle.prepre_state = vehicle._state = 'STOP'
            vehicle.brake()
            return None

        if command is None:
            vehicle.condition_yaw(angle)

        if command is not None and command != vehicle._state:
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

    def context_deta():
        context = {'Head2Target': 0,
                   'Epsilon': 0,
                   'State': 'STOP',
                   'pre': 'STOP',
                   'prepre': 'STOP',
                   'Distance': 100}
        return context

    return context_alpha()


def mqtt_RTO2(client, topic, message, userdata, blocking=False):
    topic2id = {context_topic: 'full_id', control_topic: 'semi_id'}
    retry = 5
    interval = 1
    key_mid = topic2id[topic]
    for times in range(retry):
        userdata[key_mid] = userdata[key_mid] + 1
        data = str(userdata[key_mid]) + " " + message
        print('Send message:({})'.format(data))
        infot = client.publish(topic, data, qos=2)
        if blocking:
            infot.wait_for_publish()
        for i in range(interval * 10):
            if userdata['code']:
                userdata['code'] = False
                return
            time.sleep(.1)
        print('Retry No.%d times' % (times + 1))
    userdata['code'] = False
    print('Mqtt broker has no reponses')


def mqtt_RTO(client, topic, message, userdata, blocking=False):
    topic2id = {context_topic: 'full_id', control_topic: 'semi_id'}
    key_mid = topic2id[topic]
    userdata[key_mid] = userdata[key_mid] + 1
    data = str(userdata[key_mid]) + " " + message
    logger.debug('Send message:({})'.format(data))
    infot = client.publish(topic, data, qos=2)
    if blocking:
        infot.wait_for_publish()


if __name__ == '__main__':
    client = init_mqtt(None)
    client.loop_forever()
