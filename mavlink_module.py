#!/usr/bin/evn python
#coding:utf-8

import serial
from config import config
from library import open_serial,encode_hex,encode_10h

class mavutil(object):
    def __init__(self):
        tmp=config().get_mavlink()
        portname=tmp[0]
        baud=tmp[1]
        print "Connecting to mavlink"
        self.ser = open_serial(portname,baud)

    def code_pwm(self,channels):
        msg="AABBCC"
        for channel in channels:
            msg+=encode_10h(channel)
        msg+="DD"
        return msg
        
    def send_pwm(self,channels):
        self._log(channels)
        msg=self.code_pwm(channels)
        print msg
        msg=msg.decode("hex")
        print msg
        length=self.ser.write(msg)
        return length

    def send_msg(self,msg):
        self._log(msg)
        msg=msg.decode("hex")
        length=self.ser.write(msg)
        return length

    def read_serial(self,size=63):
        while True:
            msg=self.ser.read(size)
            if msg.find('AABBCC')>0:
                n=msg.find('AABBCC')+6
                msg=msg[n:n+24]
                break
            else:
                # self._log('Read channels PWM again!')
                continue
        return encode_hex(msg).split()

    def read_channels(self):
        channels=[0,0,0,0,0,0,0,0]
        package=self.read_serial()
        print package
        n=0
        index=0
        while n<8:
            channels[n]=self.encode_pwm([package[index],package[index+1],package[index+2]])
            n+=1
            index+=3
        return channels
    def encode_pwm(self,arr):
        return int(arr[0],16)*100+int(arr[1],16)*10+int(arr[2],16)

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()
    def _log(self,msg):
        print msg
        pass


if __name__=="__main__":
    mav=mavutil()
    # channels=[1500,1500,1500,1500,1000,0,0,1500]
    # length=mav.send_pwm(channels)
    # msg='aabbcc400400400400400400400400dd'
    # mav.send_msg(msg)
    print mav.read_channels()