#!/usr/bin/evn python
#coding:utf-8

import serial,time
from library import open_serial,encode_hex
from config import config
from library import Singleton

class Compass(object):
    __metaclass__=Singleton
    def __init__(self):
        self._log("Connecting to Compass Module")
        con=config.get_compass()       
        self.ser = open_serial(con[1],con[2])
        self.state=1    # 1:healthy -1:not healthy
    def info(self):
        return '{}'.format(self.state)

    def get_heading(self):
        command='6804000307'   
        package=self.compass_info(command,83,8)
        if package==None:
            return None
        else:
            heading=self.decode_BCD(package[8:14])
            return int(heading%360)
    def get_pitch(self):
        command='6804000105'   
        package=self.compass_info(command,81,8)
        if package==None:
            return None
        else:
            pitch=self.decode_BCD(package[8:14])
            return pitch
    def get_roll(self):
        command='6804000206'
        package=self.compass_info(command,82,8)
        if package==None:
            return None
        else:
            roll=self.decode_BCD(package[8:14])
            return roll
    def get_attitude(self):
        command='6804000408'
        package=self.compass_info(command,84,14)
        if package==None:
            return None
        else:
            pitch=self.decode_BCD(package[8:14])
            yaw=self.decode_BCD(package[14:20])
            roll=self.decode_BCD(package[20:26])
            return [pitch,yaw,roll]
    def compass_info(self,command,ack,size=8):  
        command=command.decode("hex")
        times=0
        while times<config.get_compass()[3]:
            times+=1
            self.ser.write(command)
            res=self.ser.readline()
            package=encode_hex(res)
            index=package.find('68')
            if index==-1 or len(package)<index+size*2:
                continue
            package=package[index:index+size*2]
            # self._log(package)
            if package[6:8]==str(ack):
                return package
        self._log('Compass Timeout(5 times)')
        return None
    def checksum(self,package):
        pass
    def decode_BCD(self,package):
        sign=package[0]
        data=int(package[1:])/100.0
        if sign=='1':
            data=-data
        return data

    def close(self):
        if self.ser.is_open is True:
            self.ser.close()
    def _log(self,msg):
        print msg
# Global compass
compass=Compass()

if __name__=='__main__':
    while True:
        #raw_input('Next') 
        #print compass.get_pitch()
        #print compass.get_roll()
        print compass.get_heading()
        #print compass.get_attitude()
        time.sleep(.1)
