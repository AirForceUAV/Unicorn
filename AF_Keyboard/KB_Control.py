#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import keyboard
import time
from lib.logger import logger
from lib.tools import exe_actions
import protobuf.oa_rpc_pb2 as oa


keyboard_event = {'w': oa.FORWARD, 's': oa.BACKWARD,
                  'a': oa.LEFT_ROLL, 'd':oa.RIGHT_ROLL,
                  'page up': oa.UP, 'page down': oa.DOWN,
                  'space': oa.STOP, 
                  'q':oa.LEFT_YAW,'e':oa.RIGHT_YAW,
                  'esc': 'esc'}


def keyboard_event_wait():
    global keyboard_event
    valid_event = [()]
    event = keyboard.read_key(keyboard_filter)
    
    if event.event_type == 'up':
        name = 'space'
    else:
        name = event.name
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
    