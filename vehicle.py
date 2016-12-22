#!/usr/bin/evn python
# coding:utf-8

import time
import json
from config import config
from library import CancelWatcher, list2list
from library import get_distance_metres, angle_heading_target
from library import Singleton, _angle
from attribute import Attribute


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, mcu=None, ORB=None):
        super(Vehicle, self).__init__(mcu, ORB)

    def control_stick(self, AIL=0, ELE=0, THR=0, RUD=0):
        self.channels[self.AIL[0]] = self.AIL[2 + AIL * self.AIL[5]]
        self.channels[self.ELE[0]] = self.ELE[2 + ELE * self.ELE[5]]
        self.channels[self.THR[0]] = self.THR[2 + THR * self.THR[5]]
        self.channels[self.RUD[0]] = self.RUD[2 + RUD * self.RUD[5]]
        if self._frame == 'HELI':
            self.channels[self.PIT[0]] = self.THR2PIT(
                self.channels[self.THR[0]])
        self.send_pwm()

    def control_FRU(self, AIL=0, ELE=0, THR=0, RUD=0):
        self.movement(self.ELE, ELE)
        self.movement(self.AIL, AIL)
        self.movement(self.THR, THR)
        self.movement2(self.RUD, RUD)
        if self._frame == 'HELI':
            self.channels[self.PIT[0]] = self.THR2PIT(
                self.channels[self.THR[0]])
        self.send_pwm()

    def arm(self):
        print "Waiting for arming..."
        self.control_stick(1, -1, -1, -1)
        time.sleep(2)

    def disarm(self):
        self.control_stick(THR=-1)

    def takeoff(self, alt=5):
        if not self.has_module('Baro'):
            print 'Baro is closed'
            return 0
        print 'Takeoff to ', alt, 'm'

        self.control_FRU(THR=1)

        # time.sleep(2)
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            currentAlt = self.get_alt()
            print 'Current Altitude', currentAlt
            if currentAlt is None:
                break
            if currentAlt > alt * 0.9:
                print 'Reached Altitude'
                break

        self.brake()
        pass

    def land(self):
        if not self.has_module('Baro'):
            print 'Baro is closed'
            return
        print 'Landing... '

        # self.control_FRU(0, 0, -1)
        # watcher = CancelWatcher()
        # time.sleep(3)
        # preAlt = self.get_alt()
        # times = 0
        # while not watcher.IsCancel():
        #     currentAlt = self.get_alt()
        #     print 'Current Altitude', currentAlt
        #     if currentAlt is None:
        #         self.brake()
        #         return -1

        #     if abs(currentAlt - preAlt) < 0.2:
        #         times += 1
        #     else:
        #         times = 0
        #     if times >= 5:
        #         break
        #     if currentAlt > alt * 0.9:
        #         print 'Reached Altitude'
        #         break
        # if not watcher.IsCancel():
        #     self.disarm()

    def LoiterPWM(self):
        list2list(self.channels, self.channels_mid)

    def movement(self, att, sign=1):
        rate = self.gear[self.get_gear()] / 100.0
        variation = int(att[5] * att[4] * rate)
        self.channels[att[0]] = att[2] + sign * variation
        return self.channels[att[0]]

    def movement2(self, att, sign=1):
        rate = att[6] / 100.0
        variation = int(att[5] * att[4] * rate)
        self.channels[att[0]] = att[2] + sign * variation
        return self.channels[att[0]]

    def yaw_left(self):
        self._log('Turn Left...')
        self.control_FRU(RUD=-1)

    def yaw_right(self):
        self._log('Turn Right...')
        self.control_FRU(RUD=1)

    def forward(self):
        self._log('Forward...')
        self.control_FRU(ELE=1)

    def brake(self):
        self._log('brake')
        self.LoiterPWM()
        self.send_pwm()
        time.sleep(1)

    def yaw_left_brake(self):
        self._log('Yaw Left')
        moveTime = self.MoveTime()
        self.control_FRU(RUD=-1)
        time.sleep(moveTime)
        self.brake()

    def yaw_right_brake(self):
        self._log('Yaw Right')
        moveTime = self.MoveTime()
        self.control_FRU(RUD=1)
        time.sleep(moveTime)
        self.brake()

    def forward_brake(self):
        self._log('Forward')
        moveTime = self.MoveTime()
        self.control_FRU(ELE=1)
        time.sleep(moveTime)
        self.brake()

    def backward_brake(self):
        self._log('Backward')
        moveTime = self.MoveTime()
        self.control_FRU(ELE=-1)
        time.sleep(moveTime)
        self.brake()

    def roll_left_brake(self):
        self._log('Roll Left')
        moveTime = self.MoveTime()
        self.control_FRU(AIL=-1)
        time.sleep(moveTime)
        self.brake()

    def roll_right_brake(self):
        self._log('Roll Right')
        moveTime = self.MoveTime()
        self.control_FRU(AIL=1)
        time.sleep(moveTime)
        self.brake()

    def up_brake(self):
        self._log('Throttle Up')
        moveTime = self.MoveTime()
        self.control_FRU(THR=1)
        time.sleep(moveTime)
        self.brake()

    def down_brake(self):
        duration = self.MoveTime()
        self._log('Throttle Down')
        moveTime = self.MoveTime()
        self.control_FRU(THR=-1)
        time.sleep(moveTime)
        self.brake()

    def MoveTime(self):
        return self.MD

    def send_pwm(self):
        # print self.channels
        print self.analysis_channels()
        self.publish('ChannelsOutput', self.channels)
        if not self.has_module('MCU'):
            return
        self.mcu.send_pwm(self.channels)

    def analysis_channels(self):
        a = [x - y for x, y in zip(self.channels, self.channels_mid)]
        return [x * y for x, y in zip(a, self.Phase())]

    def isStop(self, begin, end, sign):
        if begin is None:
            return True
        diff = (360 + sign * (end - begin)) % 360
        return False if diff < 180 and diff > 5 else True

    def condition_yaw(self, heading=0, relative=True):
        """
        0<=heading<360
        """
        if heading < 0 or heading >= 360:
            self._log('Warning:not 0<=heading<360')
            return -1
        if relative and heading == 0:
            return

        watcher = CancelWatcher()
        current_heading = self.get_heading()
        if current_heading is None:
            return
        # Relative angle to heading
        target_angle = (current_heading +
                        heading) % 360 if relative else heading

        direction = (360 + target_angle - current_heading) % 360
        if direction > 0 and direction < 180:
            is_cw = 1
            self._log('Turn right {}'.format(direction))
            self.yaw_right()
        elif direction >= 180 and direction < 360:
            is_cw = -1
            self._log('Turn left {}'.format(360 - direction))
            self.yaw_left()
        print "Target", target_angle
        while not watcher.IsCancel() and not self.isStop(
                self.get_heading(), target_angle, is_cw):
            # self._log('Cur angle:{},Target
            # angle:{}'.format(self.get_heading(),target_angle))
            time.sleep(.01)
        self._log('Before Angle {}'.format(self.get_heading()))
        self.brake()
        self._log('After  Angle {}'.format(self.get_heading()))
        return 1

    def navigation(self, target):
        IgnoreDegree = config.get_degree()[0]
        checktime = config.get_DD()[0]
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            current_location = self.get_location()
            if current_location is None:
                break
            remain_distance = round(
                get_distance_metres(current_location, target), 2)
            self._log("Distance to Target {}m".format(remain_distance))
            if distance < 3:
                self._log("Reached Target Waypoint!")
                break
            current_yaw = self.get_heading()
            if current_yaw is None:
                break
            angle = angle_heading_target(
                current_location, target, current_yaw)
            if _angle(angle) > IgnoreDegree:
                self.brake()
                self.condition_yaw(angle)
            self.forward()
            time.sleep(checktime)
        self.brake()

    def RTL(self):
        target = self.get_home()
        if target is None:
            self._log("Home is None!")
            return
        self.publish('Mode', 'RTL')

        self.navigation(target)
        # self.land()
        self.publish('Mode', 'Loiter')

    def Route(self, info):
        self.wp.Route(info)
        # self.Auto()

    def Auto(self):
        if self.wp.isNull():
            self._log('Waypoint is None')
            return -1
        self.publish('Mode', 'Auto')
        watcher = CancelWatcher()
        for point in self.wp.remain_wp():
            if watcher.IsCancel():
                self.publish('Mode', 'Loiter')
                return 0
            self.navigation(point)
            self.wp.add_number()

        self.publish('Mode', 'Loiter')
        self.wp.clear()

    def Guided(self):
        target = self.get_target()
        if target is None:
            self._log("Target is None!")
            return -1
        self.publish('Mode', 'GUIDED')

        self.navigation(target)
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

    def Cancel(self):
        self._log("Cancel")
        CancelWatcher.Cancel = True
        time.sleep(.1)
        self.brake()

if __name__ == "__main__":
    from uORB import uORB
    from library import Watcher
    mcu = None
    Watcher()
    ORB = uORB()
    # ORB.open('Compass')
    # instancce of MCU module object
    if ORB.has_module('MCU'):
        from MCU_module import MCU
        mcu = MCU()

    if ORB.has_module('Compass'):
        # instancce of compass module object
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            # print compass.get_heading()
            time.sleep(.5)

    if ORB.has_module('GPS'):
        from GPS_module import GPS      # instancce of GPS module object
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    if ORB.has_module('Baro'):
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            # print gps.get_num_stars()
            time.sleep(.5)

    vehicle = Vehicle(mcu, ORB)
    # vehicle.arm()
    # vehicle.set_channels_mid()
    # vehicle.disarm()
    # vehicle.set_gear(2)
    # vehicle.takeoff(3)
    # vehicle.yaw_left_brake()
    # vehicle.yaw_right_brake()
    # vehicle.roll_left_brake()
    # vehicle.roll_right_brake()
    # vehicle.forward_brake()
    # vehicle.backward_brake()
    # vehicle.up_brake()
    # vehicle.down_brake()
    # vehicle.yaw_left()
    # vehicle.yaw_right()
    # vehicle.forward()
    vehicle.control_FRU(1, 1, 1)
    time.sleep(1)
    vehicle.brake()
    # vehicle.condition_yaw(30)
    # vehicle.condition_yaw(300)

    # vehicle.set_target(0, 10)
    # vehicle.download()
    # print ORB.subscribe('Waypoint')
    # vehicle.Auto()
    # vehicle.Guided()
    # vehicle.disarm()
    # print vehicle.Phase()
