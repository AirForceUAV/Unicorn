#!/usr/bin/evn python
#coding:utf-8

import serial,time,string
from library import open_serial,encode_hex
from config import config

class compass(object):
    def __init__(self):
        tmp=config().get_compass()
        portname=tmp[0]
        baud=tmp[1]
        print "Connecting to Compass Module"
        self.ser = open_serial(portname,baud)

    def get_heading(self):
        command='6804000307'   
        package=self.compass_info(command,8)
        heading=self.decode_heading(package)
        return heading
    def get_pitch(self):
        command='6804000105'   
        package=self.compass_info(command,8)
        pitch=self.decode_pitch(package)
        return pitch
    def get_roll(self):
        command='6804000206'   
        package=self.compass_info(command,8)
        roll=self.decode_roll(package)
        return roll

    def compass_info(self,command,size):  
        command=command.decode("hex")
        n=self.ser.write(command)
        res=self.ser.read(size)
        res=encode_hex(res)
        return res.split()
        
    def decode_heading(self,package):
        sign=package[4][0]
        data=int(package[4][1])*100+int(package[5])
        if sign=='1':
            data=-data
        return data
    def decode_pitch(self,package):
        sign=package[4][0]
        data=int(package[4][1])*100+int(package[5])+round(int(package[6])/100,2)
        if sign=='1':
            data=-data
        return data
    def decode_roll(self,package):
        sign=package[4][0]
        data=int(package[4][1])*100+int(package[5])+round(int(package[6])/100,2)
        if sign=='1':
            data=-data
        return data

if __name__=='__main__':
    compass=compass()
    print compass.get_heading()
    print compass.get_pitch()
    print compass.get_roll()
