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

    def Guided_Avoid(self):        
        target=vehicle.get_target()
        if target is None:
            self._log("Target is None!")
            return -1
        vehicle.mode_name='Guided_Avoid'
        vehicle._log('Guided with Avoidance to Location {}'.format(target))

        vehicle.Avoid(target)  
        vehicle.target=None
        vehicle.mode_name='Loiter'
        return 0

    def Auto_Avoid(self):
        if vehicle.wp == []:
            self._log('Waypoint is none')
            return -1
        vehicle.mode_name='AUTO_Avoid'
        watcher=CancelWatcher()
        for point in wp:
            if watcher.IsCancel():
                break
            vehicle.cur_wp+=1
            self._log("Target is None!")
            self.Avoid(point)
        
        vehicle.mode_name="Loiter"
        vehicle.clear()

    def Avoid(self,target,way=1):
        checktime=1
        deviation=config.get_degree()[1]

        watcher=CancelWatcher()
        while not watcher.IsCancel():
            current_location =vehicle.get_location()
            if current_location == None:
                self._log("GPS is ERROR!")
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
                angle_avoid=self.more_angle(angle_avoid)
                vehicle.condition_yaw(angle_avoid)
            vehicle.forward()
            time.sleep(checktime)

    def RTL_Avoid(self):
        target=vehicle.get_home()
        if target is None:
            self._log("Home is None!")
            return -1

        vehicle.mode_name='RTL_Avoid'
        self._log('RTL with Avoid! Home is {}'.format(target))
        self.Avoid(target)

        vehicle.mode_name='Loiter'
        return 0
   
    def more_angle(self,angle):
        if angle>=0 and angle<180:
            angle+=10
        else:
            angle-=10
        return angle
    def _log(self,msg):
        print msg


con=config.get_lidar()
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
        



