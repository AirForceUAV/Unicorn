#!/usr/bin/evn python
# coding:utf-8

import serial
from library import Singleton
import threading
from sbus import SBUS


class Sbus_Receiver(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB, com):
        super(Sbus_Receiver, self).__init__(name='sbus_receiver')
        self.ORB = ORB
        self._sbus = com
        self.sbus = SBUS()

    def run(self):
        print ">>> Initializing sbus_receiver ..."
        self._sbus.flushInput()
        while True:
            # package = self._sbus.read(30).encode('hex')
            package = self._sbus.readline().strip()
            # print package
            if package is '':
                continue
            package = self.sbus.filter(package)
            if package is None:
                continue
            self.ORB.publish('ChannelsInput', self.sbus.decode(package))

    def __str__(self):
        input = self.ORB.subscribe('ChannelsInput')
        return 'Input: {}'.format(input)


if __name__ == "__main__":
    from library import Watcher
    from uORB import uORB
    import time
    from tools import build_sbus
    Watcher()

    com = build_sbus()
    ORB = uORB()
    sbus_receiver = Sbus_Receiver(ORB, com)
    sbus_receiver.start()

    while ORB.subscribe('ChannelsInput') is None:
        # print sbus_receiver
        time.sleep(.5)
    while True:
        print sbus_receiver
        # raw_input('Next')
