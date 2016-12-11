#!/usr/bin/evn python
# coding:utf-8

import time
from config import config
from library import open_serial, encode_hex, encode_10h
from library import Singleton


class MCU(object):
    __metaclass__ = Singleton

    def __init__(self):
        con = config.get_MCU()
        self._log("Connecting to MCU")
        self.ser = open_serial(con[1], con[2])
        self.state = 1   # 1:healthy -1:not healthy

    def code_pwm(self, channels):
        msg = "AABB"
        for channel in channels:
            msg += encode_10h(channel)
        msg += "CC"
        return msg

    def send_pwm(self, channels):
        self._log("send pwm:{}".format(channels))
        msg = self.code_pwm(channels)
        msg = msg.decode("hex")
        self.ser.write(msg * 10)

    def read_mid(self, size=38):
        times = 0
        self.ser.flushInput()
        while times < 2:
            times += 1
            msg = self.ser.read(size * 5)
            if msg is '':
                continue
            msg = encode_hex(msg)
            # self._log("Read channels:{}".format(msg))
            n = msg.find('aabb')
            if n != -1 and len(msg) >= n + size:
                return msg[n:n + size]
        return None

    def read_channels(self, ch_count=8):

        channels = [0, 0, 0, 0, 0, 0, 0, 0]
        package = self.read_mid()
        # self._log(package)
        if package is None:
            return None
        num = 0
        index = 4
        while num < ch_count:
            channels[num] = int(package[index:index + 4], 16)
            # self._log(channels[num])
            num += 1
            index += 4
        return channels

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

    def _log(self, msg):
        print msg
        pass

if __name__ == "__main__":
    mcu = MCU()
    while True:
        ch = mcu.read_channels()
        print ch
        raw_input('Next')
