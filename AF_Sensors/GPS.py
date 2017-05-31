#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import threading
import pynmea2
from lib.tools import open_serial, Singleton
from lib.logger import logger
from lib.config import config


class GPS(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(GPS, self).__init__(name='GPS')
        self.ORB = ORB
        logger.info("Connecting to GPS Module")
        _GPS = config.GPS
        self.header = _GPS['header']
        self.ser = open_serial(_GPS['port'], _GPS['baudrate'], timeout=0.01)

    def run(self):
        logger.info("Initializing GPS Module")
        while True:
            location = self.parseGPS()
            if location is not None:
                dic = {'GPS_State': True, 'Location': self.get_location(
                    location), 'NumStars': self.get_num_stars(location)}
            else:
                dic = {'GPS_State': False, 'NumStars': 0}
            self.update(dic)
            time.sleep(.1)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def parseGPS(self):
        times = 0
        self.ser.flushInput()
        while times < 200:
            times += 1
            line = self.ser.readline()
            if line.find(self.header) != -1:
                msg = pynmea2.parse(line)
                # print line
                # return msg
                if msg.altitude is not None:
                    return msg
        return None

    def get_location(self, loc):
        return [loc.latitude, loc.longitude, loc.altitude]

    def get_num_stars(self, loc):
        return int(loc.num_sats)

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()


def GPS_start(ORB):
    gps = GPS(ORB)
    gps.start()
    while not ORB.state('GPS'):
        time.sleep(.1)
    logger.info('>>> GPS is OK')

if __name__ == "__main__":
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    GPS_start(ORB)

    while True:
        print ORB.subscribe('Location'), ORB.subscribe('NumStars')
        time.sleep(.1)
