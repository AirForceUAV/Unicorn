#!/usr/bin/evn python
# coding:utf-8

import keyboard
import time
from tools import logger


keyboard_event = {'up': 'FORWARD', 'down': 'BACKWARD',
                  'left': 'LEFT_ROLL', 'right': 'RIGHT_ROLL',
                  'page up': 'UP', 'page down': 'DOWN',
                  'space': 'STOP', 'esc': 'esc'}

map_event_args = {'FORWARD': ('ELE', 1), 'BACKWARD': ('ELE', -1),
                  'LEFT_ROLL': ('AIL', -1), 'RIGHT_ROLL': ('AIL', 1),
                  'UP': ('THR', 1), 'DOWN': ('THR', -1),
                  'LEFT_YAW': ('RUD', -1), 'RIGHT_YAW': ('RUD', 1),
                  'STOP': None
                  }


def keyboard_event_wait():
    global keyboard_event

    event = keyboard.read_key(keyboard_filter)
    name = event.name
    command = keyboard_event.get(name)

    # print 'Send -->', command
    return command


def keyboard_filter(event):
    global keyboard_event
    # wait for threading start
    time.sleep(.1)
    eventType = event.event_type
    name = event.name
    if eventType == 'down' and name in keyboard_event:
        return True
    else:
        return False


def exe_cmd(vehicle, command):
    global map_event_args
    # command = 'FORWARD'
    action = {}
    for cmd in command.split(" "):
        args = map_event_args.get(cmd, None)
        if args is None or args[0] in action:
            logger.error('command is unvalid -- {}'.format(command))
            return
        action[args[0]] = args[1]

    logger.info('Execute command:{}'.format(command))
    if vehicle is not None:
        vehicle.control_FRU(**action)


def main():
    while True:
        command = keyboard_event_wait()
        if command == 'esc':
            return
        exe_cmd(None, command)

if __name__ == '__main__':
    main()
