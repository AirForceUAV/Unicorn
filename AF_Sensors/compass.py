#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import threading
from lib.tools import open_serial, Singleton
from lib.science import CutFrame
from lib.logger import logger
from lib.config import config


class Compass(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(Compass, self).__init__(name="Compass")
        # print("Connecting to Compass ...")
        self.ORB = ORB
        self.ser = open_serial(config.compass_serial, 9600, timeout=0.01)

    def run(self):
        logger.info("Initializing Compass ...")

        while True:
            attitude = self.get_attitude()

            if attitude is None:
                dic = {'Compass_State': False}
            else:
                dic = {'Compass_State': True, 'Attitude': attitude}
            self.update(dic)
            # time.sleep(.01)

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


def compass_start(ORB):
    compass = Compass(ORB)
    compass.start()
    while not ORB.state('Compass'):
        time.sleep(.1)
    logger.info('>>> Compass is OK')

if __name__ == '__main__':
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    compass_start(ORB)

    while True:
        print ORB.subscribe('Attitude')
        # time.sleep(.1)
