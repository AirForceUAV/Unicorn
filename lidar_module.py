#!/usr/bin/evn python
# coding:utf-8

import os
import struct
import sys
import time
from config import config
from library import CancelWatcher, get_distance_metres, Singleton
from library import angle_heading_target
from vehicle import Vehicle
from library import Singleton

global config


class Lidar(object):
    _pipeSet = {}
    __metaclass__ = Singleton

    def __init__(self, vehicle=None):
        replyPipe = "./Reply"
        requestPipe = "./Request"
        con = config.get_lidar()
        pid = os.fork()
        if pid == 0:
            os.execl("./ultra_simple", "ultra_simple",
                     con[1], str(con[2]), str(con[3]), "")
            exit(0)

        if ((replyPipe, requestPipe) in self.__class__._pipeSet) is False:
            self.__class__._pipeSet[(replyPipe, requestPipe)] = {}
            self.__class__._pipeSet[(replyPipe, requestPipe)][
                "Reply"] = open(replyPipe, "r")
            self.__class__._pipeSet[(replyPipe, requestPipe)][
                "Request"] = open(requestPipe, "w")
        self.request = self.__class__._pipeSet[
            (replyPipe, requestPipe)]["Request"]
        self.reply = self.__class__._pipeSet[(replyPipe, requestPipe)]["Reply"]
        self.vehicle = vehicle

    def Decision(self, targetDirection):
        targetDirection = (360 - targetDirection) % 360
        self.request.write(struct.pack("HH", targetDirection, 0))
        self.request.flush()
        pointFmt = "HHH"
        (quality, angle, distance) = struct.unpack(
            pointFmt, self.reply.read(struct.calcsize(pointFmt)))
        angle = (360 - angle) % 360
        return angle

    def Guided(self):
        if self.vehicle is None:
            return 0
        target = self.vehicle.get_target()
        if target is None:
            self._log("Target is None!")
            return -1
        self.publish('Mode', 'GUIDED_AVOID')
        self.vehicle._log('Guided  to Location {}'.format(target))

        self.vehicle.Avoid(target)
        self.publish('Target', None)
        self.publish('Mode', 'Loiter')
        return 1

    def Auto(self):
        if self.vehicle is None:
            return 0
        if self.vehicle.wp.isNull():
            self._log('Waypoint is none')
            return -1
        self.publish('Mode', 'AUTO_AVOID')
        watcher = CancelWatcher()
        waypoints = self.subscribe('Waypoint')
        for point in waypoints:
            if watcher.IsCancel():
                break
            self.Avoid(point)
            self.wp.add_number()
        self.publish('Mode', 'Loiter')
        self.vehicle.wp.clear()

    def Avoid(self, target):
        checktime = 1
        deviation = config.get_degree()[1]

        watcher = CancelWatcher()
        while not watcher.IsCancel():
            current_location = self.vehicle.get_location()
            if current_location is None:
                self._log("GPS is not health!")
                break
            distance = round(get_distance_metres(current_location, target), 2)
            self._log("Distance to Target {}m".format(distance))
            if distance < 3:
                self._log("Reached Target Waypoint!")
                break
            angle = angle_heading_target(
                current_location, target, self.vehicle.get_heading())
            angle_avoid = self.Decision(angle)
            if self.vehicle._angle(angle_avoid) > deviation:
                self.vehicle.brake()
                angle_avoid = self.more_angle(angle_avoid)
                self.vehicle.condition_yaw(angle_avoid)
            self.vehicle.forward()
            time.sleep(checktime)
        self.vehicle.brake()

    def RTL(self):
        target = self.subscribe('HomeLocation')
        if target is None:
            self._log("Home is None!")
            return -1

        self.publish('Mode', 'RTL_AVOID')
        self._log('RTL with Avoid! Home is {}'.format(target))
        self.Avoid(target)

        self.publish('Mode', 'RTL_AVOID')
        return 0

    def more_angle(self, angle):
        if angle >= 0 and angle < 180:
            angle += 10
        else:
            angle -= 10
        return angle

    def publish(self, topic, value):
        self.vehicle.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.vehicle.ORB.subscribe(topic)

    def _log(self, msg):
        print msg

if __name__ == "__main__":
    lidar = Lidar()
    while True:
        print lidar.Decision(0)
        time.sleep(1)
