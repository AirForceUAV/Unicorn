#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import serial
import time
import threading
from lib.tools import Singleton
from sbus import SBUS
from lib.logger import logger
from lib.config import config


class Sbus_Receiver(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB, com):
        super(Sbus_Receiver, self).__init__(name='sbus_receiver')
        self.ORB = ORB
        self._sbus = com
        self.sbus = SBUS()
        # self.before = ORB.InitChannels()

    def run(self):
        logger.info("Initializing sbus_receiver ...")
        self.publish('Sbus_State', True)
        while True:
            self._sbus.flushInput()
            try:
                package = self._sbus.read(50).encode('hex')
                # package = self._sbus.readline().encode('hex').strip()
            except serial.SerialException:
                info = sys.exc_info()
                logger.error("{0}:{1}".format(*info))
            # print 'package', package

            if package is '':
                continue

            sbusFrame = self.sbus.filter(package)

            # print sbusFrame
            if sbusFrame is None:
                continue

            input = self.sbus.decode(sbusFrame)

            # if not self.check(input):
            #     continue
            self.publish('ChannelsInput', input)
            # time.sleep(.01)

    def check(self, channel):
        Flag = True
        for x, y in zip(channel, config.volume):

            if not (x > y[0] - 10 and x < y[2] + 10):
                Flag = False
                break
        return Flag

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def __str__(self):
        input = self.subscribe('ChannelsInput')
        return 'Input: {}'.format(input)


def sbus_receive_start(ORB, com=None):
    if com is None:
        from sbus import build_sbus
        com = build_sbus()
    sbus_receiver = Sbus_Receiver(ORB, com)
    sbus_receiver.start()
    while not ORB.state('Sbus') or ORB._HAL['ChannelsInput'] == None:
        time.sleep(.1)

if __name__ == "__main__":
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    sbus_receive_start(ORB)
    from AF_ML.Curve import THR2PIT
    while True:
        print sbus_receiver
        # input = ORB.subscribe('ChannelsInput')
        # print input[5], THR2PIT(input[2]), input[5] - THR2PIT(input[2])
        # raw_input('Next')
