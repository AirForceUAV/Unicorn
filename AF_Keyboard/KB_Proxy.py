import sys
sys.path.append('..')
import time
import keyboard
import pygame as pg
import paho.mqtt.client as mqtt
from lib.config import config
import AF_Avoid.oa_rpc_pb2 as action

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

    pg.init()
    display = pg.display.set_mode((100, 100))
    clock = pg.time.Clock()

    while True:
        ACK = False

        command = keyboard_state()
        if command == 'exit':
            print 'Exit'
            sys.exit(1)
            pg.quit()

        # if command != '0':
        #     print 'command', command
        print 'command', command
        infot = client.publish(config.keyboard_topic, command, qos=2)
        infot.wait_for_publish()
        while not ACK:
            clock.tick(15)
            # time.sleep(.01)
        # clock.tick(15)


def listen_keyboard2(client):
    from KB_Control import keyboard_event_wait
    global ACK
    print('start listen keboard')
    while True:
        ACK = False
        command = keyboard_event_wait()
        if command == 'esc':
            print 'Exit listen keyboard'
            return
        print 'command', command
        infot = client.publish(config.keyboard_topic, str(command), qos=2)
        infot.wait_for_publish()
        while not ACK:
            time.sleep(.01)


def keyboard_state():
    filter_key = {pg.K_SPACE: action.STOP,
                  pg.K_w: action.FORWARD,
                  pg.K_s: action.BACKWARD,
                  pg.K_q: action.LEFT_YAW,
                  pg.K_e: action.RIGHT_YAW,
                  pg.K_a: action.LEFT_ROLL,
                  pg.K_d: action.RIGHT_ROLL,
                  pg.K_PAGEUP: action.UP,
                  pg.K_PAGEDOWN: action.DOWN,
                  pg.K_ESCAPE: 'exit'}
    pg.event.pump()
    pressed = pg.key.get_pressed()

    if pressed[pg.K_ESCAPE] == 1:
        return 'exit'

    keys = []
    for i in xrange(len(pressed)):
        if pressed[i] == 1 and (i in filter_key):
            keys.append(filter_key[i])
    if keys == []:
        keys = [action.STOP]
    command = ','.join(map(str, keys))
    return command


def start_client(host, port, id):
    client = mqtt.Client(client_id=id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port)
    client.loop_start()
    return client

if __name__ == '__main__':
    sock = config.KB_socket
    print sock
    args = (sock[0], sock[1], config.keyboard_topic)
    # args = ('localhost', 1883, config.keyboard_topic)
    client = start_client(*args)
    listen_keyboard(client)
   
