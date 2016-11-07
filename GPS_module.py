#!/usr/bin/evn python
#coding:utf-8

import pynmea2,time
from config import config
from library import open_serial
from library import Singleton,Watcher
import threading

class GPS(threading.Thread):
    __metaclass__=Singleton
    def __init__(self):
        threading.Thread.__init__(self)
        self._log("Connecting to Compass Module")
        con=config.get_GPS()
        self.ser = open_serial(con[1],con[2])
        self.state=1  # 1:healthy -1:not healthy
        self.msg=None
        
    def run(self):
        print "Initializing GPS Module"
        while True:
            self.msg=self.parseGPS()
            
    def info(self):
        return '{},{}'.format(self.state,self.get_num_stars())

    def parseGPS(self):
        times=0
        while times<200:
            times+=1
            line=self.ser.readline()

            if line.find('GNGGA')!=-1:
                msg=pynmea2.parse(line)
                if msg.altitude!=None:
                    return msg
        print "GPS timeout!"
        return None

    def get_location(self):    
        msg=self.msg
        if msg==None:
            return None
        else:
            return [msg.latitude,msg.longitude,msg.altitude]

    def get_num_stars(self):
        msg=self.msg
        if msg==None:
            return 0
        else:
            return int(msg.num_sats)
        
    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

    def _log(self,msg):
        print msg

if __name__=="__main__":
    gps=GPS()
    Watcher()
    gps.start()
    while gps.msg==None:
        time.sleep(.5)
    while True:
        print gps.get_location()
        # print gps.get_num_stars()
        time.sleep(.1)


  
