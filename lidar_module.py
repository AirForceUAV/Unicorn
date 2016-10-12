#!/usr/bin/evn python
#coding:utf-8

import os,struct,sys,time
from config import config
from library import CancelWatcher,get_distance_metres,Singleton
from vehicle import vehicle
from library import Singleton

global vehicle
global config

class Lidar(object):
    _pipeSet = {}
    __metaclass__=Singleton
    def __init__(self):
        replyPipe="./Reply"
        requestPipe="./Request"
        con=config.get_lidar()

        if self.__class__._pipeSet.has_key((replyPipe,requestPipe)) is False:
            self.__class__._pipeSet[(replyPipe,requestPipe)] = {}
            self.__class__._pipeSet[(replyPipe,requestPipe)]["Reply"] =  open(replyPipe,"r")
            self.__class__._pipeSet[(replyPipe,requestPipe)]["Request"] =  open(requestPipe,"w")
        self.request = self.__class__._pipeSet[(replyPipe,requestPipe)]["Request"]
        self.reply= self.__class__._pipeSet[(replyPipe,requestPipe)]["Reply"]
        

    def Decision(self,targetDirection):
        targetDirection = (360 - targetDirection) % 360
        self.request.write(struct.pack("HH",targetDirection,0))
        self.request.flush()
        pointFmt = "HHH"
        (quality,angle,distance) = struct.unpack(pointFmt,self.reply.read(struct.calcsize(pointFmt)))
        angle = (360 - angle) %360;
        return angle

    def Guided_Avoid2(self,_type='Guided',checktime=5,deviation=2):        
        watcher=CancelWatcher()
        if _type is "Guided":
            target=vehicle.get_target()
            if target is None:
                self._log("Target is None!")
                return -1
            vehicle.mode_name='Guided_AVOID'
            vehicle._log('Guided with Avoidance to Location {}'.format(target))
            
        elif _type is 'RTL':
            target=vehicle.get_home()
            if target is None:
                self._log("Home is None!")
                return -1
            vehicle.mode_name='RTL_AVOID'
            self._log('RTL ! Home is {}'.format(target))

        while not watcher.IsCancel():
            current_location =vehicle.get_location()
            if current_location == None:
                print "GPS is ERROR!"
                vehicle.brake()
                break
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                vehicle.brake()
                break  
            angle=vehicle.angle_heading_target(current_location,target)
            angle_avoid=self.Decision(angle)
            if vehicle._angle(angle_avoid)>deviation:
                vehicle.brake()
                vehicle.condition_yaw(angle_avoid)
            vehicle.forward()
            time.sleep(checktime)
        vehicle.mode_name='Loiter'
        return 0

    def RTL_Avoid(self):
        self.Guided_Avoid('RTL')
    def Auto_Avoid(self):
        self.Guided_Avoid('AUTO')
    def _log(self,msg):
        vehicle._log(msg)


pid = os.fork()
    if pid == 0:
        os.execl("./ultra_simple","ultra_simple",con[1],str(con[2]),str(con[3]),"")
        exit(0)
# Global lidar       
lidar=Lidar()

if __name__ == "__main__" :
    while True:
        print lidar.Decision(0)
        time.sleep(1)
        



