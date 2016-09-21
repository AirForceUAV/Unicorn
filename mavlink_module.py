#!/usr/bin/evn python
#coding:utf-8

import serial,time
from config import config
from library import open_serial,encode_hex,encode_hex2,encode_10h,radio_package,GCS_package,Mid_package

class mavutil(object):
    def __init__(self):
        tmp=config().get_mavlink()
        portname=tmp[0]
        baud=tmp[1]
        print "Connecting to mavlink"
        self.ser = open_serial(portname,baud)

    def code_pwm(self,channels):
        msg="AABB"
        for channel in channels:
            msg+=encode_10h(channel)
        msg+="CC"
        return msg

    # def filter_channel(self,channel):
    #     channel=str(channel)
    #     length=len(channel)
    #     if length is 1:
    #         channel+='00'
    #     elif length is 2:
    #         channel+='0'
    #     return channel
       
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
            self.ser.write(msg)                 
            ack=(self.ser.read(1)).strip()
            print 'ACK',times,':',ack
            if ack=='S' and len(msg)>2:
                # print 'Send package successfully!'
                return 'S'
            elif ack=='M' and ack==msg:
                # print 'Current mode is MID'
                return 'M'
            elif ack=='G' and ack==msg:
                # print 'Current mode is GCS'
                return 'G'
            elif ack=='R'and ack==msg:
                # print 'Current mode is Radio'
                return 'R'
            elif ack=='F' and len(msg)>2:
                # print 'ARQ-F'
                time.sleep(.2)
                continue
            else:
                # print 'ARQ'
                time.sleep(.2)
                continue
        print "Timeout (10 times)"
        return -1

    def read_mid(self,size=74):      
        while True:
            # print 'Read channels Mid'
            msg=self.ser.read(size)
            msg=encode_hex2(msg)
            # print "Read channels:",msg
            if msg.find('aabb')>=0:
                n=msg.find('aabb')+4
                msg=msg[n:n+32]
                break
            else:
                # self._log('Read channels PWM again!')
                continue
        return msg

    def read_channels(self):
        self.send_msg(Mid_package())
        channels=[0,0,0,0,0,0,0,0]
        package=self.read_mid()
        num=0
        index=0
        while num<8:
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


if __name__=="__main__":
    mav=mavutil()
    # channels=[1500,1500,1500,1500,1000,0,0,1500]
    # length=mav.send_pwm(channels)
    # mav.send_msg('G')
    print mav.read_channels()