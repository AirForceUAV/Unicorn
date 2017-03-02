import paho.mqtt.client as mqtt
from keyboard_control import keyboard_event_wait
from config import config


def on_connect(client, userdata, flags, rc):
    print("Keyboard has connected to mqtt. code:{}".format(rc))


def listen_keyboard(client):
    print 'start listen keboard'
    precommand = ''
    while True:
        command = keyboard_event_wait()
        if command == 'esc':
            print 'Exit listen keyboard'
            infot = client.publish(config.keyboard_topic, command, qos=2)
            infot.wait_for_publish()
            return

        # precommand = command
        print 'keyboard_command', command
        infot = client.publish(config.keyboard_topic, command, qos=2)
        infot.wait_for_publish()


def start_client(host, port, id):
    client = mqtt.Client(client_id=id)
    client.on_connect = on_connect
    client.connect(host, port)
    client.loop_start()
    return client

if __name__ == '__main__':
    args = ('localhost', 1883, 'keyboard')
    # args = ('192.168.1.4', 12345, 'keyboard')
    client = start_client(*args)
    listen_keyboard(client)
