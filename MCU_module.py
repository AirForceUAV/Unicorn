#!/usr/bin/evn python
#coding:utf-8

import serial,time
from config import config
from library import open_serial,encode_hex,encode_10h
from library import radio_package,GCS_package,Mid_package
from library import Singleton

class MCU(object):
    __metaclass__=Singleton
    def __init__(self):
        global config
        con=config.get_MCU()
        self._log("Connecting to mavlink")
        self.ser = open_serial(con[1],con[2])
        self.state=1   # 1:healthy -1:not healthy

    def code_pwm(self,channels):
        msg="AABB"
        for channel in channels:
            msg+=encode_10h(channel)
        msg+="CC"
        return msg
       
    def send_pwm(self,channels):
        print "send pwm:",channels       
        msg=self.code_pwm(channels)
        msg=msg.decode("hex")
        times=0
        while times<config.get_MCU()[3]:
            times+=1
            self.ser.write(GCS_package())
            self.ser.write(msg)
            ack=self.ser.read(40)
            # print ack
            if ack.find('S')>=0:
                print "Send successfully"
                return 1
            else:
                time.sleep(0.05)
                continue
        print "Send package Timout ({} times)!".format(times)
        return -1

    def send_msg(self,msg):
        # Send 'R' or 'G'
        print "send msg:",msg
        times=0
        while times<5:
            times+=1
            self.ser.write(msg)            
            time.sleep(0.05)

    def send_mid_msg(self):
        # Send 'M'
        print 'send msg',Mid_package()
        times=0
        while times<config.get_MCU()[3]:
            times+=1
            self.ser.write(Mid_package())
            ack=self.ser.read(42)
            ack=msg=encode_hex(ack)
            # print ack
            if ack.find('aabb')!=-1:
                return 1
            else:
                time.sleep(0.05)
                continue

        print 'Switch to M. Timeout ({} times)'.format(times)

    def read_mid(self,size=74): 
        '''Sizeof package is (4+4*8+2=38)'''   
        while True:
            msg=self.ser.read(size)
            msg=encode_hex(msg)
            # print "Read channels:",msg
            n=msg.find('aabb')
            if n!=-1 and len(msg)>=n+38:                
                return msg[n:n+38]
               
    def read_channels(self,ch_count=8):
        self.ser.flushInput()
        self.send_mid_msg()
        channels=[0,0,0,0,0,0,0,0]
        package=self.read_mid()
        # print package
        num   = 0
        index = 4
        while num<ch_count:
            channels[num]=int(package[index:index+4],16)
            # print channels[num]
            num+=1
            index+=4           
        return channels

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()
    def _log(self,msg):
        print msg
        pass

# Global mcu
mcu=MCU()

if __name__=="__main__":
    # channels=[1500,1500,1500,1500,1000,0,0,1500]
    # mcu.send_pwm(channels)
    # mcu.send_msg('G')
    print mcu.read_channels()