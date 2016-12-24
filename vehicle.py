#!/usr/bin/evn python
# coding:utf-8

import time
import json
from library import CancelWatcher
from library import get_distance_metres, angle_heading_target
from library import Singleton, _angle
from attribute import Attribute


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, mcu=None, ORB=None):
        super(Vehicle, self).__init__(mcu, ORB)
        self.moveTime = 2
        self.brakeTime = 1

    def control_stick(self, AIL=0, ELE=0, THR=0, RUD=0):
        channels = [0] * 8
        channels[self.AIL[0]] = self.AIL[2 + AIL * self.AIL[5]]
        channels[self.ELE[0]] = self.ELE[2 + ELE * self.ELE[5]]
        channels[self.THR[0]] = self.THR[2 + THR * self.THR[5]]
        channels[self.RUD[0]] = self.RUD[2 + RUD * self.RUD[5]]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def control_FRU(self, AIL=0, ELE=0, THR=0, RUD=0):
        channels = [0] * 8
        channels[self.AIL[0]] = self.movement(self.AIL, AIL)
        channels[self.ELE[0]] = self.movement(self.ELE, ELE)
        channels[self.THR[0]] = self.movement(self.THR, THR)
        channels[self.RUD[0]] = self.movement2(self.RUD, RUD)
        self._construct_channel(channels)
        self.send_pwm(channels)

    def _construct_channel(self, channels):
        channels[self.mode[0]] = self.mode[2]
        if self._model == 'HELI':
            channels[self.Rate[0]] = self.Rate[2]
            channels[self.PIT[0]] = self.THR2PIT(
                channels[self.THR[0]])
        else:
            channels[self.Aux1[0]] = self.Aux1[2]
            channels[self.Aux2[0]] = self.Aux2[2]
        channels[self.Switch[0]] = self.Switch[2]

    def arm(self):
        self._log("Arming ...")
        self.control_stick(1, -1, -1, -1)
        time.sleep(2)

    def disarm(self):
        self._log('DisArmed ...')
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

    def movement(self, channel, sign=1):
        rate = self.ORB._Gear[self.subscribe('Gear') - 1] * 0.01
        variation = int(channel[5] * channel[4] * rate)
        return channel[2] + sign * variation

    def movement2(self, channel, sign=1):
        rate = channel[6] * 0.01
        variation = int(channel[5] * channel[4] * rate)
        return channel[2] + sign * variation

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
        self.send_pwm(self.subscribe('LoiterPWM'))
        time.sleep(self.brakeTime)

    def yaw_left_brake(self):
        self._log('Yaw Left')
        self.control_FRU(RUD=-1)
        time.sleep(self.moveTime)
        self.brake()

    def yaw_right_brake(self):
        self._log('Yaw Right')
        self.control_FRU(RUD=1)
        time.sleep(self.moveTime)
        self.brake()

    def forward_brake(self):
        self._log('Forward')
        self.control_FRU(ELE=1)
        time.sleep(self.moveTime)
        self.brake()

    def backward_brake(self):
        self._log('Backward')
        self.control_FRU(ELE=-1)
        time.sleep(self.moveTime)
        self.brake()

    def roll_left_brake(self):
        self._log('Roll Left')
        self.control_FRU(AIL=-1)
        time.sleep(self.moveTime)
        self.brake()

    def roll_right_brake(self):
        self._log('Roll Right')
        self.control_FRU(AIL=1)
        time.sleep(self.moveTime)
        self.brake()

    def up_brake(self):
        self._log('Throttle Up')
        self.control_FRU(THR=1)
        time.sleep(self.moveTime)
        self.brake()

    def down_brake(self):
        self._log('Throttle Down')
        self.control_FRU(THR=-1)
        time.sleep(self.moveTime)
        self.brake()

    def send_pwm(self, channels):
        print channels
        print self.analysis_channels(channels)
        self.publish('ChannelsOutput', channels)
        if not self.has_module('MCU'):
            return
        self.mcu.send_pwm(channels)

    def analysis_channels(self, channels):
        a = [x - y for x, y in zip(channels, self.subscribe('LoiterPWM'))]
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
        IgnoreDegree = 10
        checktime = 5
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
    ORB.open('MCU', 'Compass', 'GPS')
    ORB.close('GPS')
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
            time.sleep(.5)

    if ORB.has_module('IMU'):
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.subscribe('IMU_State'):
            time.sleep(.5)

    # ORB.start()
    vehicle = Vehicle(mcu, ORB)
    vehicle.arm()
    vehicle.set_channels_mid()
    # vehicle.set_gear(2)
    # vehicle.takeoff(3)
    vehicle.yaw_left_brake()
    vehicle.yaw_right_brake()
    vehicle.roll_left_brake()
    vehicle.roll_right_brake()
    vehicle.forward_brake()
    vehicle.backward_brake()
    vehicle.up_brake()
    vehicle.down_brake()
    # vehicle.yaw_left()
    # time.sleep(1)
    # vehicle.yaw_right()
    # time.sleep(1)
    # vehicle.forward()
    # vehicle.control_FRU(AIL=1, ELE=1)
    # time.sleep(1)
    # vehicle.brake()
    # vehicle.condition_yaw(30)
    # vehicle.condition_yaw(300)

    # vehicle.set_target(0, 10)
    # vehicle.download()
    # print ORB.subscribe('Waypoint')
    # vehicle.Auto()
    # vehicle.Guided()
    vehicle.disarm()
    # print vehicle.Phase()
