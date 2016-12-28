#!/usr/bin/evn python
# coding:utf-8

import os
import struct
import time
from library import CancelWatcher, get_distance_metres, Singleton
from library import angle_heading_target
from library import Singleton


class Lidar(object):
    _pipeSet = {}
    __metaclass__ = Singleton

    def __init__(self, vehicle=None):
        replyPipe = "./Reply"
        requestPipe = "./Request"
        safety_distance = 1500
        detect_distance = 3000
        port = '/dev/ttyUSB0'

        pid = os.fork()
        if pid == 0:
            os.execl("./ultra_simple", "ultra_simple",
                     port, safety_distance, 3000, "")
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
            return
        target = self.vehicle.get_target()
        if target is None:
            self._log("Target is None!")
            return
        self.publish('Mode', 'GUIDED_AVOID')
        self._log('Guided  to Location {}'.format(target))

        self.vehicle.Avoid(target)
        self.publish('Target', None)
        self.publish('Mode', 'Loiter')

    def Auto(self):
        if self.vehicle is None:
            return
        if self.vehicle.wp.isNull():
            self._log('Warning:Waypoint is none')
            return
        self.publish('Mode', 'AUTO_AVOID')
        watcher = CancelWatcher()
        waypoints = self.subscribe('Waypoint')
        for point in waypoints:
            if watcher.IsCancel():
                break
            self.Avoid(point)
            self.vehicle.wp.add_number()
        self.publish('Mode', 'Loiter')
        self.vehicle.wp.clear()

    def Avoid(self, target):
        checktime = 1
        IgnoreDegree = 10

        watcher = CancelWatcher()
        while not watcher.IsCancel():
            current_location = self.vehicle.get_location()
            if current_location is None:
                break
            distance = round(get_distance_metres(current_location, target), 2)
            self._log("Distance to Target {}m".format(distance))
            if distance < 3:
                self._log("Reached Target Waypoint!")
                break
            current_yaw = self.vehicle.get_heading()
            if current_yaw is None:
                break
            angle = angle_heading_target(
                current_location, target, current_yaw)
            angle_avoid = self.Decision(angle)
            if self.vehicle._angle(angle_avoid) > IgnoreDegree:
                self.vehicle.brake()
                angle_avoid = self.more_angle(angle_avoid)
                self.vehicle.condition_yaw(angle_avoid)
            self.vehicle.forward()
            time.sleep(checktime)
        self.vehicle.brake()

    def RTL(self):
        target = self.subscribe('HomeLocation')
        if target is None:
            self._log("Warning:Home is None!")
            return

        self.publish('Mode', 'RTL_AVOID')
        self._log('RTL with Avoidance! Home is {}'.format(target))
        self.Avoid(target)

        self.publish('Mode', 'Loiter')

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
        # self.vehicle._log(msg)

if __name__ == "__main__":
    lidar = Lidar()
    while True:
        print lidar.Decision(0)
        time.sleep(1)
