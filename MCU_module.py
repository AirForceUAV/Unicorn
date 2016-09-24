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

    def code_pwm(self,channels):
        msg="AABB"
        for channel in channels:
            msg+=encode_10h(channel)
        msg+="CC"
        return msg
       
    def send_pwm(self,channels):
        print "send pwm:",channels
        self.ser.write(GCS_package())
        msg=self.code_pwm(channels)
        # print msg
        msg=msg.decode("hex")
        result=self.write_serial(msg)
        return result

    def send_msg(self,msg):
        print "send msg:",msg
        # msg=msg.decode("hex")
        result=self.write_serial(msg)
        return result

    def write_serial(self,msg):
        times=0         
        while times<10:
            times+=1
            self.ser.flushInput()
            length=self.ser.write(msg)                
            ack=(self.ser.read(1)).strip()
            print 'ACK',times,':',ack
            if ack=='S' and len(msg)>6:
                # print 'Send package successfully!'
                return 1
            elif ack=='F' and len(msg)>6:
                # print 'ARQ-F'
                # time.sleep(.2)
                continue
            elif ack==msg:
                return 1
            else:
                # print 'ARQ'
                # time.sleep(.2)
                continue
        print "Timeout (10 times)"
        return -1

    def read_mid(self,size=74): 
        '''Sizeof package is (4+4*8+2=38)'''
        self.ser.flushInput()     
        while True:
            # print 'Read channels Mid'
            msg=self.ser.read(size)
            msg=encode_hex(msg)
            # print "Read channels:",msg
            if msg.find('aabb')>=0:
                n=msg.find('aabb')+4
                msg=msg[n:n+32]
                break
            else:
                # self._log('Read channels PWM again!')
                continue
        return msg

    def read_channels(self,ch_count=8):
        self.send_msg(Mid_package())
        channels=[0,0,0,0,0,0,0,0]
        package=self.read_mid()
        print package
        num   = 0
        index = 0
        while num<ch_count:
            print [package[index+1],package[index+2],package[index+3]]
            channels[num]=self.encode_pwm([package[index+1],package[index+2],package[index+3]])
            num+=1
            index+=4           
        return channels

    def encode_pwm(self,arr):
        return  int(arr[0],16)*16*16+int(arr[1],16)*16+int(arr[2],16)

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