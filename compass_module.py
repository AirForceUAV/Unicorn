#!/usr/bin/evn python
# coding:utf-8

import time
import threading
from library import open_serial
from library import Singleton
from library import CutFrame


class Compass(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(Compass, self).__init__(name="Compass")
        print ">>> Connecting to Compass ..."
        self.ORB = ORB
        self.ser = open_serial('/dev/compass', 9600, timeout=0.01)

    def run(self):
        print ">>> Initializing Compass ..."

        while True:
            attitude = self.get_attitude()

            if attitude is None:
                dic = {'Compass_State': False}
            else:
                dic = {'Compass_State': True, 'Attitude': attitude}
            self.update(dic)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def get_attitude(self):
        command = '6804000408'
        package = self.RawFrame(command, '84', 14)
        if package is None:
            return None
        else:
            pitch = self.decode_BCD(package[8:14])
            roll = self.decode_BCD(package[14:20])
            yaw = self.decode_BCD(package[20:26])
            return [pitch, roll, int(yaw)]

    def RawFrame(self, command, ack, size=8):
        command = command.decode("hex")
        times = 0
        self.ser.flushInput()
        while times < 100:
            times += 1
            self.ser.write(command)
            package = self.ser.readline().encode('hex')
            index = package.find('68')
            if index == -1 or len(package) < index + size * 2:
                continue
            package = package[index:index + size * 2]
            if package[6:8] == ack and self.checksum(package):
                return package
        return None

    def checksum(self, package):
        pieces = CutFrame(package)
        sum = reduce(lambda x, y: x + y, pieces[1:-1]) % 256
        return sum == pieces[-1]

    def decode_BCD(self, package):
        sign = package[0]
        data = int(package[1:]) / 100.0
        if sign == '1':
            data = -data
        return data

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()


if __name__ == '__main__':
    from library import Watcher
    from uORB import uORB
    ORB = uORB()
    compass = Compass(ORB)
    Watcher()
    compass.start()
    while not ORB.subscribe('Compass_State'):
        time.sleep(.1)
    print 'Compass is OK'
    while True:
        print ORB.subscribe('Attitude')
        # time.sleep(.1)
