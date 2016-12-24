#!/usr/bin/evn python
# coding:utf-8

import time
from library import open_serial, ascii2hex, dec2hex
from library import Singleton
from library import CutFrame


class MCU(object):
    __metaclass__ = Singleton

    def __init__(self):
        print '>>> Connecting to MCU ...'
        self._mcu = open_serial('/dev/ttyAMA0', 57600)
        self.FRAME_HEAD = 'AABB'
        self.FRAME_TAIL = 'CC'
        self.FRAME_LEN = 19

    def EncodeChannels(self, channels):
        msg = reduce(lambda x, y: x + dec2hex(y),
                     [self.FRAME_HEAD] + channels) + self.FRAME_TAIL
        # msg = "AABB"
        # for channel in channels:
        #     msg += dec2hex(channel)
        # msg += "CC"
        return msg

    def send_pwm(self, channels):
        msg = self.EncodeChannels(channels).decode('hex')
        self._mcu.write(msg * 10)

    def RawFrame(self):
        times = 0
        size = self.FRAME_LEN * 2
        self._mcu.flushInput()
        while times < 10:
            times += 1
            msg = self._mcu.read(size * 5)
            if msg is '':
                continue
            msg = ascii2hex(msg)
            n = msg.find('aabb')
            if len(msg) < n + size:
                continue
            if n != -1 and msg[n + size - 2: n + size] == 'cc':
                return msg[n: n + size]
        return None

    def read_channels(self):
        package = self.RawFrame()
        return None if package is None else CutFrame(package[4:36], 4)


if __name__ == "__main__":
    mcu = MCU()
    # a = [1000] * 8
    # print mcu.EncodeChannels(a)
    while True:
        ch = mcu.read_channels()
        print ch
        raw_input('Next')
