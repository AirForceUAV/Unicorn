#!/usr/bin/evn python
# coding:utf-8

import time
from config import config
from library import open_serial, encode_hex, encode_10h
from library import GCS_package, Mid_package
from library import Singleton


class MCU(object):
    __metaclass__ = Singleton

    def __init__(self):
        global config
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

    def send_msg(self, msg):
        # Send 'R' , 'G' , 'M'
        self._log("send msg:{}".format(msg))
        self.ser.write(msg * 100)

    def send_mid_msg(self):
        # Send 'M'
        self._log('send msg:{}'.format(Mid_package()))
        times = 0
        while times < config.get_MCU()[3]:
            times += 1
            self.ser.write(Mid_package())
            ack = self.ser.read(42)
            ack = encode_hex(ack)
            # print(ack)
            if ack.find('aabb') != -1:
                return 1
            else:
                time.sleep(0.05)

        self._log('Switch to M. Timeout ({} times)'.format(times))

    def read_mid(self, size=100):

        while True:
            msg = self.ser.read(size)
            msg = encode_hex(msg)
            #self._log("Read channels:{}".format(msg))
            n = msg.find('aabb')
            if n != -1 and len(msg) >= n + 38:
                return msg[n:n + 38]

    def read_channels(self, ch_count=8):
        self.ser.flushInput()
        self.send_msg('M')
        channels = [0, 0, 0, 0, 0, 0, 0, 0]
        package = self.read_mid()
        # self._log(package)
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
    # channels=[1500,1500,1500,1500,1000,0,0,1500]
    # mcu.send_pwm(channels)
    # mcu.send_msg('G')
    print mcu.read_channels()
