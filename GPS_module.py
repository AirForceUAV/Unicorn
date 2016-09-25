#!/usr/bin/evn python
#coding:utf-8

import serial,pynmea2,time
from config import config
from library import open_serial,get_distance_metres,get_location_metres
from library import Singleton

class GPS(object):
    __metaclass__=Singleton
    def __init__(self):
        con=config.get_GPS()
        print "Connecting to GPS Module"
        self.ser = open_serial(con[1],con[2])
        self.reader= pynmea2.NMEAStreamReader()

    def parseGPS(self): 
        while  True:
            self.ser.flushInput()
            line=self.ser.readline()
            if line.find('GNGGA')>0:
                msg=pynmea2.parse(line)
                return msg
            else:
                continue

    def get_location(self):
        times=0
        while times<3:
            times+=1
            msg=self.parseGPS()
            print msg
            if msg.latitude==0 and msg.longitude==0:
                continue
            else:
                return [round(msg.latitude,5),round(msg.longitude,5)]
                # return [int(msg.lat)/100.0,int(msg.lon)/100.0]
        # print "GPS is not healthy.[Debug]:num_stars is",self.get_num_stars()
        return None
        

    def get_alt(self):
        msg=self.parseGPS()
        return msg.altitude

    def get_num_stars(self):
        msg=self.parseGPS()
        return msg.num_sats

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

# Global gps
gps=GPS()

if __name__=="__main__":
    print 1
    origin=gps.get_location()
    print 2
    # print 'origin:{}'.format(origin)
    # # assert origin!=None
    target=get_location_metres(origin,0,-20)
    # distance=get_distance_metres(origin,target)
    # print 'origin:{} Target:{},distance:{}'.format(origin,target,distance)
    while True:
      raw_input('Next')
      loc=gps.get_location()     
      distance=get_distance_metres(loc,target)
      # print 'cur_location:{},num_stars:{}'.format(loc,gps.get_num_stars())
      print 'Distance to Target {}'.format(distance)
      if distance<2:
        print 'reached'
        break
    gps.close()
