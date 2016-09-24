#!/usr/bin/evn python
#coding:utf-8

import serial,time
from config import config
from library import open_serial,encode_hex

class MPU(object):	
	def __init__(self):
		tmp=config().get_MPU()
		portname=tmp[0]
		baud=tmp[1]
		print "Connecting to MPU Module"
		self.ser = open_serial(portname,baud)
	def get_alt(self):
		while True:
			res=self.ser.read(50)
			res=encode_hex(res)
			if res.find('55')>=0:
				return res.split()
			else:
				continue
		# return encode_hex(self.ser.read(30))

if __name__=='__main__':
	mpu=MPU()
	print mpu.get_alt()
