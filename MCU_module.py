#!/usr/bin/evn python
# coding:utf-8

import time
from config import config
from library import open_serial, ascii2hex, dec2hex
from library import Singleton
from library import ParseFrame


class MCU(object):
    __metaclass__ = Singleton

    def __init__(self):
        con = config.get_MCU()
        self._log("Connecting to MCU")
        self.ser = open_serial(con[1], con[2])
        self.FRAME_HEAD = 'AABB'
        self.FRAME_TAIL = 'CC'

    def code_pwm(self, channels):
        msg = reduce(lambda x, y: x + dec2hex(y),
                     [self.FRAME_HEAD] + channels) + self.FRAME_TAIL
        # msg = "AABB"
        # for channel in channels:
        #     msg += dec2hex(channel)
        # msg += "CC"
        return msg

    def send_pwm(self, channels):
        msg = self.code_pwm(channels)
        msg = msg.decode("hex")
        self.ser.write(msg * 10)

    def get_frame(self, size=38):
        times = 0
        self.ser.flushInput()
        while times < 10:
            times += 1
            msg = self.ser.read(size * 5)
            if msg is '':
                continue
            msg = ascii2hex(msg)
            # self._log("Read channels:{}".format(msg))
            n = msg.find('aabb')
            if len(msg) < n + size:
                continue
            if n != -1 and msg[n + size - 2: n + size] == 'cc':
                return msg[n: n + size]
        return None

    def read_channels(self):
        package = self.get_frame()
        # self._log(package)
        return None if package is None else ParseFrame(package[4:36], 4)

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

    def _log(self, msg):
        print msg
        pass

if __name__ == "__main__":
    mcu = MCU()
    a = [1000] * 8
    print mcu.code_pwm(a)
    # while True:
    #     ch = mcu.read_channels()
    #     print ch
    #     raw_input('Next')
