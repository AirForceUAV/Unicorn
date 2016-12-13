#!/usr/bin/evn python
# coding:utf-8

import time
import threading
from library import open_serial, ascii2hex
from config import config
from library import Singleton


class Compass(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(Compass, self).__init__(name="Compass")
        self._log("Connecting to Compass Module")
        self.ORB = ORB
        con = config.get_compass()
        self.ser = open_serial(con[1], con[2])

    def run(self):
        print "Initializing Compass Module"

        while True:
            attitude = self.get_attitude()

            if attitude is None:
                dic = {'Compass_State': -1, 'Attitude': None}
            else:
                dic = {'Compass_State': 1, 'Attitude': attitude}
            self.update(dic)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def get_attitude(self):
        command = '6804000408'
        package = self.compass_info(command, 84, 14)
        if package is None:
            return None
        else:
            pitch = self.decode_BCD(package[8:14])
            roll = self.decode_BCD(package[14:20])
            heading = int(self.decode_BCD(package[20:26]))
            return [pitch, roll, heading]

    def compass_info(self, command, ack, size=8):
        command = command.decode("hex")
        times = 0
        self.ser.flushInput()
        while times < 100:
            times += 1
            self.ser.write(command)
            res = self.ser.readline()
            package = ascii2hex(res)
            index = package.find('68')
            if index == -1 or len(package) < index + size * 2:
                continue
            package = package[index:index + size * 2]
            # self._log(package)
            if package[6:8] == str(ack) and self.checksum(package):
                return package
        self._log('Compass Timeout')
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
    from library import Watcher
    from uORB import uORB
    ORB = uORB()
    compass = Compass(ORB)
    Watcher()
    compass.start()
    while ORB.subscribe('Compass_State') is -1:
        time.sleep(.5)
    while True:
        print ORB.subscribe('Attitude')
        time.sleep(.3)
