#!/usr/bin/evn python
# coding:utf-8

import paho.mqtt.client as mqtt
from debug_env import *


def init_mqtt(userdata=None):
    client = mqtt.Client(client_id='gan')
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(host, port)
    return client


def on_connect(client, userdata, flags, rc):
    print("Connected cloud with result code {}".format(rc))
    client.subscribe([(context_topic, 2), (control_topic, 2)])


def on_message(client, userdata, msg):
    map_topic = {context_topic: full_auto_topic,
                 control_topic: semi_auto_topic}
    message = msg.payload
    topic = msg.topic
    print('Recv message:{}'.format(message))
    client.publish(map_topic[topic], message, qos=2)

if __name__ == '__main__':
    client = init_mqtt()
    client.loop_forever()
