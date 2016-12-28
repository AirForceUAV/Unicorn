#!/usr/bin/evn python
# coding:utf-8

import time
import json
import math
from library import CancelWatcher
from library import get_distance_metres, angle_heading_target
from library import Singleton, _angle
from attribute import Attribute


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, sbus_sender=None, ORB=None):
        super(Vehicle, self).__init__(sbus_sender, ORB)
        self.moveTime = 2
        self.brakeTime = 0.5

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
            channels[self.PIT[0]] = self.PIT[2]
        else:
            channels[self.Aux1[0]] = self.Aux1[2]
            channels[self.Aux2[0]] = self.Aux2[2]
        channels[self.Switch[0]] = self.Switch[2]

    def arm(self):
        self._log("Arming ...")
        self.control_stick(1, -1, -1, -1)
        time.sleep(1)
        self.disarm()

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
        # self._log('Turn Left...')
        self.control_FRU(RUD=-1)

    def yaw_right(self):
        # self._log('Turn Right...')
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
        self.sbus_sender.send_pwm(channels)

    def analysis_channels(self, channels):
        a = [x - y for x, y in zip(channels, self.subscribe('LoiterPWM'))]
        return [x * y for x, y in zip(a, self.Phase())]

    def isStop(self, heading, target, sign):
        diff = (360 + sign * (heading - target)) % 360
        return False if diff <= 180 and diff > 5 else True

    def condition_yaw(self, heading=0):
        """
        0<=heading<360 (anti-clockwise)
        """
        if heading <= 0 or heading >= 360:
            return

        watcher = CancelWatcher()
        current_yaw = self.get_heading()
        if current_yaw is None:
            return
        # Relative angle to heading
        target_angle = (360 + current_yaw - heading) % 360

        TurnAngle = (360 + current_yaw - target_angle) % 360
        if TurnAngle >= 0 and TurnAngle <= 180:
            is_cw = 1
            self._log('Turn left {}'.format(TurnAngle))
            self.yaw_left()
        else:
            is_cw = -1
            self._log('Turn right {}'.format(360 - TurnAngle))
            self.yaw_right()
        print "Target", target_angle
        while not watcher.IsCancel():
            current_yaw = self.get_heading()
            if current_yaw is None:
                return
            if self.isStop(current_yaw, target_angle, is_cw):
                break
            # self._log('{},{}'.format(current_yaw, target_angle))
            time.sleep(.1)
        self._log('Before:{}'.format(self.get_heading()))
        self.brake()
        self._log('After:{}'.format(self.get_heading()))

    def navigation(self, target):
        watcher = CancelWatcher()
        radius = 5
        frequency = 0.5
        current_location = self.get_location()
        current_yaw = self.get_heading()
        if current_location is None or current_yaw is None or target is None:
            return

        init_angle = angle_heading_target(
            current_location, target, current_yaw)
        self.condition_yaw(init_angle)

        while not watcher.IsCancel():
            current_location = self.get_location()
            current_yaw = self.get_heading()
            if current_location is None or current_yaw is None:
                break
            distance = round(
                get_distance_metres(current_location, target), 2)
            angle = angle_heading_target(
                current_location, target, current_yaw)

            print 'Distance :', distance

            if not self.InAngle(angle, 90) or angle < radius + 5:
                self._log("Reached Target Waypoint!")
                break

            SAngle = int(math.degrees(math.asin(radius / distance))) + 20
            if not self.InAngle(angle, SAngle):
                self.brake()
                self.condition_yaw(angle)
            self.forward()
            time.sleep(frequency)
        self.brake()

    def _navigation(self, target):
        watcher = CancelWatcher()
        radius = 5
        frequency = 0.5
        current_location = self.get_location()
        current_yaw = self.get_heading()
        if current_location is None or current_yaw is None or target is None:
            return

        init_angle = angle_heading_target(
            current_location, target, current_yaw)
        self.condition_yaw(init_angle)

        while not watcher.IsCancel():
            current_location = self.get_location()
            current_yaw = self.get_heading()
            if current_location is None or current_yaw is None:
                break
            distance = round(
                get_distance_metres(current_location, target), 2)
            angle = angle_heading_target(
                current_location, target, current_yaw)

            print 'Distance :', distance

            if not self.InAngle(angle, 90) or angle < radius + 5:
                self._log("Reached Target Waypoint!")
                break
            SAngle = int(math.degrees(math.asin(radius / distance))) + 20

            if self.InAngle(angle, SAngle):
                self.forward()
            else:
                if angle > SAngle and angle <= 90:
                    print 'Roll Left'
                    self.control_FRU(AIL=-1, ELE=1)
                elif angle >= 270 and angle < 360 - SAngle:
                    print 'Roll Right'
                    self.control_FRU(AIL=1, ELE=1)
                else:
                    self.condition_yaw(angle)
            time.sleep(frequency)
        self.brake()

    def InAngle(self, angle, SAngle):
        if angle < 360 - SAngle and angle > SAngle:
            return False
        else:
            return True

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

    ORB = uORB()
    Watcher()

    sbus_sender = None

    if ORB.has_module('MCU'):
        # Initialize SBUS
        from sbus_receiver import Sbus_Receiver
        from sbus_sender import Sbus_Sender
        from tools import build_sbus

        com = build_sbus()
        sbus_receiver = Sbus_Receiver(ORB, com)
        sbus_receiver.start()

        while ORB.subscribe('ChannelsInput') is None:
            time.sleep(.5)

        sbus_sender = Sbus_Sender(ORB, com)
        sbus_sender.start()
        time.sleep(.5)

    if ORB.has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.subscribe('Compass_State'):
            time.sleep(.5)

    if ORB.has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.subscribe('GPS_State'):
            time.sleep(.5)

    if ORB.has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.subscribe('Baro_State'):
            time.sleep(.5)

    if ORB.has_module('IMU'):
        # Initialize IMU
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.subscribe('IMU_State'):
            time.sleep(.5)

    # Save FlightLog to SD card
    # ORB.start()

    # Initialize UAV
    vehicle = Vehicle(sbus_sender, ORB)

    # Test
    import sys
    from tools import commands
    Test = commands
    for t in Test:
        enter = raw_input(t + '???').strip()
        if enter is '':
            command = 'vehicle.' + t
            print 'Execute command ->', command
            try:
                eval(command)
            except Exception:
                info = sys.exc_info()
                print "{0}:{1}".format(*info)
        elif enter == 'c':
            CancelWatcher.Cancel = True
            vehicle.brake()
            break
        else:
            continue
    print 'Completed'
