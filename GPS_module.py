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
        self.state=1  # 1:healthy -1:not healthy
        
    def info(self):
        return '{},{}'.format(self.state,self.get_num_stars())

    def parseGPS(self):
        times=0
        while times<config.get_GPS()[3]:
            self.ser.flushInput()
            line=self.ser.readline()
            if line.find('GNGGA')!=-1:
                msg=pynmea2.parse(line)
                if msg.altitude!=None:
                    return msg
        return None

    def get_location(self):    
        msg=self.parseGPS()
        if msg==None:
            return None
        else:
            return [round(msg.latitude,5),round(msg.longitude,5),msg.altitude]

    def get_alt(self):
        msg=self.parseGPS()
        if msg==None:
            return None
        else:
            return msg.altitude

    def get_num_stars(self):
        msg=self.parseGPS()
        if msg==None:
            return None
        else:
            return msg.num_sats
        
    def close(self):
        if self.ser.is_open is True:
            self.ser.close()

# Global gps
gps=GPS()

if __name__=="__main__":
    print gps.get_alt()
    # origin=gps.get_location()
    # # print 'origin:{}'.format(origin)
    # # # assert origin!=None
    # target=get_location_metres(origin,0,-20)
    # # distance=get_distance_metres(origin,target)
    # # print 'origin:{} Target:{},distance:{}'.format(origin,target,distance)
    # while True:
    #   raw_input('Next')
    #   loc=gps.get_location()     
    #   distance=get_distance_metres(loc,target)
    #   # print 'cur_location:{},num_stars:{}'.format(loc,gps.get_num_stars())
    #   print 'Distance to Target {}'.format(distance)
    #   if distance<2:
    #     print 'reached'
    #     break
    # gps.close()
