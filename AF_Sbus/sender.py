#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import threading
from sbus import SBUS
from lib.science import ascii2hex, dec2hex, CutFrame
from lib.tools import Singleton, CancelWatcher
from lib.logger import logger
from lib.config import config


class Sbus_Sender(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB, com):
        super(Sbus_Sender, self).__init__(name='sbus_sender')
        self.ORB = ORB
        self._sbus = com
        self.IsRadio = True
        self.sbus = SBUS()
        self.index = 0

    def run(self):
        logger.info('Initializing sbus_sender ...')

        switch = config.channels['Switch']
        GCS_PWM = switch[2]
        while True:
            package = self.subscribe('ChannelsInput')
            if package is None:
                continue
            ch8 = package[switch[0]]
            # print ch8
            if ch8 < GCS_PWM + 100 and ch8 > GCS_PWM - 100:
                # GCS Mode
                # print 'GCS'
                if self.IsRadio:
                    self.IsRadio = False
                    self.publish('ChannelsOutput', package)
                output = self.subscribe('ChannelsOutput')

                if output is None:
                    continue
                # print output
                self.send_package(output)
            else:
                # Radio Mode
                # print 'Radio'
                CancelWatcher.Cancel = True
                self.IsRadio = True
                self.send_package(package)
            # time.sleep(.01)
            # self.send_package(package)

    def send_package(self, package):
        FRAME_TAIL = self.sbus.END_BYTE[self.index]
        package = self.sbus.encode(package) + FRAME_TAIL
        self._sbus.write(package.decode('hex'))
        self.index = (self.index + 1) % 4

    def EncodeChannels(self, channels):
        msg = reduce(lambda x, y: x + dec2hex(y),
                     ['aabb'] + channels) + 'cc'
        return msg

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def __str__(self):
        input = self.subscribe('ChannelsInput')
        output = self.subscribe('ChannelsOutput')
        return "Input: {} Output: {}".format(input, output)


def sbus_start(ORB):
    from receiver import sbus_receive_start
    from sbus import build_sbus

    com = build_sbus()
    sbus_receive_start(ORB, com)

    sbus_sender = Sbus_Sender(ORB, com)
    sbus_sender.start()

    logger.info('Sbus is OK')


if __name__ == "__main__":
    from AF_uORB.uORB import uORB
    from lib.tools import Watcher

    ORB = uORB()
    Watcher()

    sbus_start(ORB)

    while True:
        input = ORB.subscribe('ChannelsInput')
        output = ORB.subscribe('ChannelsOutput')
        print "Input:{} Output:{}".format(input, output)
