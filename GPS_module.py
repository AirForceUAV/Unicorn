#!/usr/bin/evn python
#coding:utf-8

import serial,pynmea2,time
from config import config
from library import open_serial

class GPS(object):	
	def __init__(self):
		tmp=config().get_GPS()
		portname=tmp[0]
		baud=tmp[1]
		print "Connecting to GPS Module"
		self.ser = open_serial(portname,baud)
		self.reader= pynmea2.NMEAStreamReader()

	def parseGPS(self):		
		while  True:
			line=self.ser.readline()
			if line.find('GGA')>=0:
				print "Catching GPS information successfully !"
				msg=pynmea2.parse(line)
				return msg
			else:
				print "GPS is error..."
				time.sleep(0.1)
				continue

	def get_location(self):
		msg=self.parseGPS()
		print msg
		return [msg.latitude,msg.longitude]

	def get_alt(self):
		msg=self.parseGPS()
		return msg.altitude

	def get_num_sats(self):
		msg=self.parseGPS()
		return msg.num_sats

	def close(self):
		if self.ser.is_open is True:
			self.ser.close()

if __name__=="__main__":
	gps=GPS()
	print gps.get_location()
	print gps.get_alt()
	print get_num_sats()
	gps.close()