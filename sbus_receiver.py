#!/usr/bin/evn python
# coding:utf-8

import serial
from library import Singleton
import threading
from sbus import SBUS
import sys
import time


class Sbus_Receiver(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB, com):
        super(Sbus_Receiver, self).__init__(name='sbus_receiver')
        self.ORB = ORB
        self._sbus = com
        self.sbus = SBUS()
        self.before = ORB.InitChannels()

    def run(self):
        print ">>> Initializing sbus_receiver ..."
        while True:
            self._sbus.flushInput()
            try:
                package = self._sbus.read(50).encode('hex')
                # package = self._sbus.readline().strip()
            except serial.SerialException:
                info = sys.exc_info()
                print "{0}:{1}".format(*info)
            # print 'package', package

            if package is '':
                continue

            sbusFrame = self.sbus.filter(package)

            # print sbusFrame
            if sbusFrame is None:
                continue

            input = self.sbus.decode(sbusFrame)

            if not self.check(input):
                continue
            self.publish('Sbus_State', True)
            self.publish('ChannelsInput', input)

    def check(self, channels):
        for x, y in zip(channels, self.before):
            if abs(x - y) > 200:
                self.before = channels
                return False

        self.before = channels
        return True

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def __str__(self):
        input = self.subscribe('ChannelsInput')
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

    while not ORB.state('Sbus'):
        time.sleep(.1)
    while True:
        print sbus_receiver
        # raw_input('Next')
        # time.sleep(1)

    # sbus = SBUS()
    # with open('Curve.ML', 'a+') as f:
    #     while True:
    #         input = ORB.subscribe('ChannelsInput')
    #         # print input
    #         line = "{},{}\n".format(input[2], input[5])
    #         print line
    #         f.write(line)
    #         time.sleep(.1)
