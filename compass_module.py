#!/usr/bin/evn python
# coding:utf-8

import time
import threading
from library import open_serial, encode_hex, Watcher
from config import config
from library import Singleton


class Compass(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self):
        threading.Thread.__init__(self)
        self._log("Connecting to Compass Module")
        con = config.get_compass()
        self.ser = open_serial(con[1], con[2])
        self.state = 1    # 1:healthy -1:not healthy
        self.attitude = None

    def run(self):
        print "Initializing Compass Module"
        while True:
            self.attitude = self._attitude()

    def info(self):
        return '{}'.format(self.state)

    def _attitude(self):
        command = '6804000408'
        package = self.compass_info(command, 84, 14)
        if package == None:
            return None
        else:
            pitch = self.decode_BCD(package[8:14])
            roll = self.decode_BCD(package[14:20])
            heading = int(self.decode_BCD(package[20:26]))
            return [pitch, roll, heading]

    def get_attitude(self):
        return self.attitude

    def get_pitch(self):
        return self.attitude[0]

    def get_roll(self):
        return self.attitude[1]

    def get_heading(self):
        return self.attitude[2]

    def compass_info(self, command, ack, size=8):
        command = command.decode("hex")
        times = 0
        while times < config.get_compass()[3]:
            times += 1
            self.ser.write(command)
            res = self.ser.readline()
            package = encode_hex(res)
            index = package.find('68')
            if index == -1 or len(package) < index + size * 2:

                continue
            package = package[index:index + size * 2]
            # self._log(package)
            if package[6:8] == str(ack):
                return package
        self._log('Compass Timeout(5 times)')
        return None

    def checksum(self, package):
        return True

    def decode_BCD(self, package):
        sign = package[0]
        data = int(package[1:]) / 100.0
        if sign == '1':
            data = -data
        return data

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

    def _log(self, msg):
        print msg

if __name__ == '__main__':
    compass = Compass()

    Watcher()
    compass.start()
    while compass.attitude == None:
        time.sleep(.5)
    while True:
        print compass.get_attitude()
        time.sleep(.1)
