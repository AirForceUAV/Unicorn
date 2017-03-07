#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import os
import toml
import keyboard

exit = False


def change_exit():
    global exit
    exit = True


def collect_pwm(ORB, topic):
    global exit
    print('Start collecting')
    topic_file = os.path.join('..', 'ML', topic + '.pwm')

    keyboard.add_hotkey('esc', change_exit)
    channels = []
    pre = []
    while not exit:
        input = ORB.subscribe('ChannelsInput')
        # print input
        if input != pre:
            channels.append(input)
            pre = input
    exit = False
    message = toml.dumps({topic: channels})
    with open(topic_file, 'w') as f:
        f.write(message)

    print('End collecting')


if __name__ == '__main__':
    topic = 'STAB'
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB
    from AF_Sbus.receiver import sbus_receive_start

    ORB = uORB()
    Watcher()

    sbus_receive_start(ORB)

    print('Sbus receiver is OK')
    collect_pwm(ORB, topic)
