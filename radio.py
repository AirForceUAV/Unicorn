#!/usr/bin/evn python
# coding:utf-8

from library import ascii2hex, dec2hex
from library import Singleton
from library import CutFrame
import threading
from library import CancelWatcher


class Sbus_Sender(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB, com):
        super(Sbus_Sender, self).__init__(name='sbus_sender')
        self.ORB = ORB
        self._sbus = com
        self.FRAME_HEAD = b'aabb'
        self.FRAME_TAIL = b'cc'
        self.FRAME_LEN = 19
        self.TIMEOUT = 100
        self.IsRadio = True

    def run(self):
        print '>>> Initializing sbus_sender ...'
        while True:
            package = self.subscribe('ChannelsInput')
            if package is None:
                continue
            self.send_package(package)

    def send_package(self, channels):
        # self._sbus.flushOutput()
        # print channels
        if self.ORB._model['Model'] == "HELI":
            channels = map(lambda x: int(
                1012 + (x - 352) * 775 / 1344), channels)
        else:
            channels = map(lambda x: int(
                1101 + (x - 352) * 843 / 1344), channels)
        # print channels
        self._sbus.write(self.EncodeChannels(channels))

    def EncodeChannels(self, channels):
        msg = reduce(lambda x, y: x + dec2hex(y),
                     [self.FRAME_HEAD] + channels) + self.FRAME_TAIL
        # msg = "AABB"
        # for channel in channels:
        #     msg += dec2hex(channel)
        # msg += "CC"
        return msg.decode('hex')

    def send_pwm(self, channels):
        if not self.IsRadio:
            self.publish('ChannelsOutput', channels)

    def read_channels(self):
        return self.subscribe('ChannelsInput')

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def __str__(self):
        info = [
            self.subscribe('ChannelsInput'),
            self.subscribe('ChannelsOutput')]
        return "Input: {} Output: {}".format(*info)

if __name__ == "__main__":
    import time
    from uORB import uORB
    from library import Watcher
    from sbus_receiver import Sbus_Receiver
    from tools import build_sbus

    com = build_sbus('/dev/ttyUSB0')
    ORB = uORB()
    Watcher()
    sbus_receiver = Sbus_Receiver(ORB, com)
    sbus_receiver.start()

    sbus_sender = Sbus_Sender(ORB, com)
    sbus_sender.start()
    # while True:
    #     print sbus_sender
