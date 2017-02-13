#!/usr/bin/evn python
# coding:utf-8

import pynmea2
import time
from library import open_serial
from library import Singleton
import threading
from tools import _log


class GPS(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(GPS, self).__init__(name='GPS')
        self.ORB = ORB
        _log("Connecting to GPS Module")
        self.ser = open_serial('/dev/GPS', 9600, timeout=0.01)

    def run(self):
        _log("Initializing GPS Module")
        while True:
            location = self.parseGPS()
            if location is not None:
                dic = {'GPS_State': True, 'Location': self.get_location(
                    location), 'NumStars': self.get_num_stars(location)}
            else:
                dic = {'GPS_State': False, 'NumStars': 0}
            self.update(dic)
            time.sleep(.01)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def parseGPS(self):
        times = 0
        self.ser.flushInput()
        while times < 200:
            times += 1
            line = self.ser.readline()
            if line.find('GNGGA') != -1:
                msg = pynmea2.parse(line)
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

if __name__ == "__main__":
    from library import Watcher
    from uORB import uORB
    ORB = uORB()
    gps = GPS(ORB)
    Watcher()
    gps.start()
    while not ORB.subscribe('GPS_State'):
        time.sleep(.1)
    print 'GPS is OK'
    while True:
        print ORB.subscribe('Location'), ORB.subscribe('NumStars')
        # time.sleep(.1)
