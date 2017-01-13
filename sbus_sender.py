#!/usr/bin/evn python
# coding:utf-8

from library import ascii2hex, dec2hex
from library import Singleton
from library import CutFrame
import threading
from library import CancelWatcher
import time
from sbus import SBUS


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
        print '>>> Initializing sbus_sender ...'
        self.publish('Sender_State', True)
        switch = self.ORB._channel['Switch']
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
            # self.send_package(package)

    def send_package(self, package):
        # self._sbus.flushOutput()
        # print channels
        # if self.ORB._model['Model'] == "HELI":
        #     channels = map(lambda x: int(
        #         1012 + (x - 352) * 775 / 1344), channels)
        # else:
        #     channels = map(lambda x: int(
        #         1101 + (x - 352) * 843 / 1344), channels)
        # print channels
        # self._sbus.write(self.EncodeChannels(channels).decode('hex'))
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

if __name__ == "__main__":
    import time
    from uORB import uORB
    from library import Watcher
    from sbus_receiver import Sbus_Receiver
    from tools import build_sbus

    com = build_sbus()
    ORB = uORB()
    Watcher()
    sbus_receiver = Sbus_Receiver(ORB, com)
    sbus_receiver.start()

    while not ORB.state('Sbus'):
        time.sleep(.1)

    sbus_sender = Sbus_Sender(ORB, com)
    sbus_sender.start()

    while not ORB.state('Sender'):
        time.sleep(.1)
    while True:
        print sbus_sender
        # time.sleep(1)
