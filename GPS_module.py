#!/usr/bin/evn python
# coding:utf-8

import pynmea2
import time
from config import config
from library import open_serial
from library import Singleton
import threading


class GPS(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(GPS, self).__init__(name='GPS')
        self.ORB = ORB
        self._log("Connecting to Compass Module")
        con = config.get_GPS()
        self.ser = open_serial(con[1], con[2])
        self.init()

    def run(self):
        print "Initializing GPS Module"
        while True:
            location = self.parseGPS()
            if location is not None:
                dic = {'GPS_State': 1, 'Location': self.get_location(
                    location), 'Num_stars': self.get_num_stars(location)}
            else:
                dic = {'GPS_State': -1, 'Location': None,
                       'Num_stars': 0}
            self.update(dic)

    def init(self):
        dic = {'GPS_State': -1, 'Location': None, 'Num_stars': 0}
        self.update(dic)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def parseGPS(self):
        times = 0
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

    def _log(self, msg):
        print msg

if __name__ == "__main__":
    from library import Watcher
    from uORB import uORB
    ORB = uORB()
    gps = GPS(ORB)
    Watcher()
    gps.start()
    while ORB.subscribe('GPS_State') is -1:
        time.sleep(.5)
    while True:
        print ORB.subscribe('Location')
        print ORB.subscribe('Num_stars')
        time.sleep(.1)
