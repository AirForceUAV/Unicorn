#!/usr/bin/evn python
# coding:utf-8

import paho.mqtt.client as mqtt
from config import config


def init_mqtt(userdata=None):
    client = mqtt.Client(client_id='lidar')
    client.user_data_set(userdata)
    client.on_connect = on_connect  # callback when connected
    client.on_message = on_message  # callback when received message
    client.connect(*config.mqtt_socket)
    return client


def on_connect(client, userdata, flags, rc):
    print("Connected cloud with result code {}".format(rc))
    client.subscribe([(config.context_topic, 2),
                      (config.control_topic, 2)])


def on_message(client, userdata, msg):
    map_topic = {config.context_topic: config.full_auto_topic,
                 config.control_topic: config.semi_auto_topic}
    message = msg.payload
    topic = msg.topic
    print('From topic:[{}] receive message:({})'.format(topic, message))
    if topic == config.context_topic:
        message = message.split(" ")[0] + " FORWARD"
    topic = map_topic[topic]
    print('Send message:({}) to topic:[{}]'.format(message, topic))
    client.publish(topic, message, qos=2)

if __name__ == '__main__':
    client = init_mqtt()
    client.loop_forever()
