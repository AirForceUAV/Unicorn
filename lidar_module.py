#!/usr/bin/evn python
#coding:utf-8

import os,struct,sys,time
from config import config
from library import CancelWatcher,get_distance_metres,Singleton,angle_heading_target
from library import Singleton

global config

class Lidar(object):
    _pipeSet = {}
    __metaclass__=Singleton
    def __init__(self,vehicle):
        replyPipe="./Reply"
        requestPipe="./Request"
        self.vehicle=vehicle
        con=config.get_lidar()

        pid = os.fork()
        if pid == 0:
            os.execl("./ultra_simple","ultra_simple",con[1],str(con[2]),str(con[3]),"")
            exit(0)

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
        target=self.vehicle.get_target()
        if target is None:
            self._log("Target is None!")
            return -1
        self.vehicle.mode_name='Guided_Avoid'
        self.vehicle._log('Guided with Avoidance to Location {}'.format(target))

        self.vehicle.Avoid(target)  
        self.vehicle.target=None
        self.vehicle.mode_name='Loiter'
        return 0

    def Auto_Avoid(self):
        if self.vehicle.wp == []:
            self._log('Waypoint is none')
            return -1
        self.vehicle.mode_name='AUTO_Avoid'
        watcher=CancelWatcher()
        for point in wp:
            if watcher.IsCancel():
                break
            self.vehicle.cur_wp+=1
            self._log("Target is None!")
            self.Avoid(point)
        
        self.vehicle.mode_name="Loiter"
        self.vehicle.clear()

    def Avoid(self,target,way=1):
        checktime=config.get_DD()[1]
        deviation=config.get_degree()[1]

        watcher=CancelWatcher()
        while not watcher.IsCancel():
            current_location =self.vehicle.get_location()
            if current_location == None:
                self._log("GPS is ERROR!")
                self.vehicle.brake()
                break
            distance=round(get_distance_metres(current_location,target),2)
            self._log("Distance to Target {}m".format(distance))
            if distance<3:
                self._log("Reached Target Waypoint!")
                self.vehicle.brake()
                break  
            angle=angle_heading_target(current_location,target,self.vehicle.get_heading())
            angle_avoid=self.Decision(angle)
            if self.vehicle._angle(angle_avoid)>deviation:
                self.vehicle.brake()
                angle_avoid=self.more_angle(angle_avoid)
                self.vehicle.condition_yaw(angle_avoid)
            self.vehicle.forward()
            time.sleep(checktime)

    def RTL_Avoid(self):
        target=self.vehicle.get_home()
        if target is None:
            self._log("Home is None!")
            return -1

        self.vehicle.mode_name='RTL_Avoid'
        self._log('RTL with Avoid! Home is {}'.format(target))
        self.Avoid(target)

        self.vehicle.mode_name='Loiter'
        return 0
   
    def more_angle(self,angle):
        if angle>=0 and angle<180:
            angle+=10
        else:
            angle-=10
        return angle
    def _log(self,msg):
        print msg

if __name__ == "__main__" :
    lidar=Lidar()
    while True:
        print lidar.Decision(0)
        time.sleep(1)
        



