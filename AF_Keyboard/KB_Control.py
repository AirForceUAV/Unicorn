#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import keyboard
import time
from lib.logger import logger

# keyboard_event = {'w': 'FORWARD', 's': 'BACKWARD',
#                   'a': 'LEFT_ROLL', 'd': 'RIGHT_ROLL',
#                   'page up': 'UP', 'page down': 'DOWN',
#                   'space': 'STOP', 'esc': 'esc'}

keyboard_event = {'w': '1', 's': '2',
                  'a': '16', 'd': '32',
                  'page up': '64', 'page down': '128',
                  'space': '0', 
                  'q':'4','e':'32',
                  'esc': 'esc'}

map_event_args = {'FORWARD': ('ELE', 1), 'BACKWARD': ('ELE', -1),
                  'LEFT_ROLL': ('AIL', -1), 'RIGHT_ROLL': ('AIL', 1),
                  'UP': ('THR', 1), 'DOWN': ('THR', -1),
                  'LEFT_YAW': ('RUD', -1), 'RIGHT_YAW': ('RUD', 1),
                  'STOP': None
                  }


def keyboard_event_wait():
    global keyboard_event
    valid_event = [()]
    event = keyboard.read_key(keyboard_filter)
    event_info = [event.name, event.event_type]

    if event_info[1] == 'up':
        name = 'space'
    else:
        name = event_info[0]
    # print name
    command = keyboard_event.get(name)

    # print 'keboard command', command
    return command


def keyboard_filter(event):
    global keyboard_event
    # wait for threading start
    eventType = event.event_type
    name = event.name
    # print name
    if name in keyboard_event:
        return True
    else:
        return False


def exe_cmd(vehicle, command):
    global map_event_args
    action = {}
    for cmd in command.strip().split(" "):
        if cmd == 'STOP':
            action = {}
            break
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
        print 'command', command
        if command == 'esc':
            print'esc'
            return

        # exe_cmd(None, command)

if __name__ == '__main__':
    main()
