import sys
sys.path.append('..')
import time
import paho.mqtt.client as mqtt
from KB_Control import keyboard_event_wait, keyboard_event
from lib.config import config

ACK = False


def on_connect(client, userdata, flags, rc):
    print("Keyboard has connected to mqtt. code:{}".format(rc))
    client.subscribe('ACK', qos=2)


def on_message(client, userdata, msg):
    global ACK
    # print 'Receive', msg.payload
    ACK = True


def listen_keyboard(client):
    global ACK
    print 'start listen keboard'
    precommand = ''
    while True:
        ACK = False
        command = keyboard_event_wait()
        if command == 'esc':
            print 'Exit listen keyboard'
            return
        print 'command', command
        infot = client.publish(config.keyboard_topic, command, qos=2)
        infot.wait_for_publish()
        while not ACK:
            time.sleep(.01)


def start_client(host, port, id):
    client = mqtt.Client(client_id=id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port)
    client.loop_start()
    return client

if __name__ == '__main__':
    args = ('localhost', 1883, config.keyboard_topic)
    # args = ('192.168.1.4', 12345, config.keyboard_topic)
    client = start_client(*args)
    listen_keyboard(client)
