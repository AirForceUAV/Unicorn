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

	def parseGPS(self,size=100):	
		while  True:
			self.ser.flushInput()
			line=self.ser.read(size)
			if line.find('GGA')>=0:
				msg=pynmea2.parse(line)
				return msg
			else:
				continue

	def get_location(self):
		times=0
		while times<3:
			times+=1
			msg=self.parseGPS()
			if msg.latitude==0 and msg.longitude==0:
				continue
			else:
				return [msg.latitude,msg.longitude]
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
	loc=gps.get_location()
	print loc
	target=get_location_metres(loc,0,10)
	while True:
		raw_input('Next')
		loc=gps.get_location()
		distance=get_distance_metres(loc,target)
		print 'cur_location:{} Target_location:{},{}'.format(loc,target,gps.get_num_stars())
		print 'Distance to Target {}'.format(distance)
	gps.close()