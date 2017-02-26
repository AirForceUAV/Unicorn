#!/usr/bin/evn python
# coding:utf-8

import time
import json
import math
from library import CancelWatcher
from library import get_distance_metres, angle_heading_target
from library import Singleton, _angle
from attribute import Attribute
from Curve import THR2PIT
from config import *
from tools import logger


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(Vehicle, self).__init__(ORB)
        self.moveTime = 2
        self.Epsilon = 20
        self.radius = 5
        self.prepre_state = 'STOP'
        self.pre_state = 'STOP'
        self._state = 'STOP'
        self._target = None

    def brake(self, braketime=0.5):
        self.send_pwm(self.subscribe('LoiterPWM'))
        time.sleep(braketime)

    def control_stick(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=3):
        channels = [0] * 8
        channels[self.AIL[0]] = self.AIL[2 + AIL * self.AIL[5]]
        channels[self.ELE[0]] = self.ELE[2 + ELE * self.ELE[5]]
        channels[self.THR[0]] = self.THR[2 + THR * self.THR[5]]
        channels[self.RUD[0]] = self.RUD[2 + RUD * self.RUD[5]]
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def control_FRU(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=3):
        channels = [0] * 8
        channels[self.AIL[0]] = self.movement(self.AIL, AIL)
        channels[self.ELE[0]] = self.movement(self.ELE, ELE)
        channels[self.THR[0]] = self.movement2(self.THR, THR)
        channels[self.RUD[0]] = self.movement2(self.RUD, RUD)
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def control_percent(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=3):
        channels = [0] * 8
        channels[self.AIL[0]] = self.movement3(self.AIL, AIL)
        channels[self.ELE[0]] = self.movement3(self.ELE, ELE)
        channels[self.THR[0]] = self.control_THR(THR)
        channels[self.RUD[0]] = self.movement3(self.RUD, RUD)
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def _construct_channel(self, channels):
        if drone['Model'] == 'HELI':
            channels[self.Rate[0]] = self.Rate[2]
            channels[self.PIT[0]] = THR2PIT(channels[self.THR[0]])
            # channels[self.PIT[0]] = self.PIT[0]
        else:
            channels[self.Aux1[0]] = self.Aux1[2]
            channels[self.Aux2[0]] = self.Aux2[2]
        channels[self.Switch[0]] = self.Switch[2]

    def movement(self, channel, sign=1):
        # sign in [-1,0,1]. By Gear
        gear = self.subscribe('Gear') - 1
        rate = drone['Gear'][gear] / 100.0
        index = 2 + channel[5] * sign
        section = abs(channel[2] - channel[index])
        variation = int(channel[5] * section * rate)
        return channel[2] + sign * variation

    def movement2(self, channel, sign=1):
        # sign in [-1,0,1].  By XML
        rate = channel[6] / 100.0
        index = 2 + channel[5] * sign
        section = abs(channel[2] - channel[index])
        variation = int(channel[5] * section * rate)
        return channel[2] + sign * variation

    def movement3(self, channel, percent=0):
        # -100 <= percent <= 100. By Percent of PWM
        sign = 0
        if percent < 0:
            sign = -1
        elif percent > 0:
            sign = 1
        rate = percent / 100.0
        index = 2 + channel[5] * sign
        section = abs(channel[2] - channel[index])
        variation = int(channel[5] * section * rate)
        result = channel[2] + variation
        return result

    def GradualTHR(self, begin, end):
        if begin <= end:
            while begin <= end:
                self.control_percent(THR=begin)
                begin += 1
                time.sleep(0.05)
        else:
            while begin >= end:
                self.control_percent(THR=begin)
                begin -= 1
                time.sleep(0.05)

    def control_THR(self, percent):
        # 0 <= percent <= 100
        THR = self.THR
        rate = percent / 100.0
        section = THR[4]
        if THR[5] < 0:
            rate = 1 - rate
        variation = int(section * rate)
        result = THR[1] + variation
        return result

    def arm(self):
        logger.info("Arming ...")
        if drone['Model'] == 'HELI':
            return
        self.control_stick(-1, -1, -1, 1)
        # time.sleep(2)
        # self.disarm()

    def disarm(self):
        logger.info('DisArmed ...')
        self.control_stick(THR=-1, Mode=2)

    def takeoff(self, alt=5):
        watcher = CancelWatcher()
        logger.info('Takeoff to {} m'.format(alt))
        if not self.has_module('Baro'):
            logger.warn('Baro is closed')
            return

        self.escalate(0, 60)

        while not watcher.IsCancel():
            currentAlt = self.get_alttitude(True)
            logger.debug('Current Altitude :{}'.format(currentAlt))
            if currentAlt is None:
                logger.error('Baro is not health')
                break
            if currentAlt > alt * 0.95:
                logger.info('Reached Altitude :{}'.format(currentAlt))
                break
            time.sleep(.1)

        self.brake()

    def land(self):
        logger.info('Landing...')
        if not self.has_module('Baro'):
            logger.warn('Baro is closed')
            return

        # self.control_FRU(0, 0, -1)
        # watcher = CancelWatcher()
        # time.sleep(3)
        # preAlt = self.get_altitude()
        # times = 0
        # while not watcher.IsCancel():
        #     currentAlt = self.get_altitude()
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

    def up_metres(self, altitude, relative=True):
        if altitude <= 0:
            logger.warn('Altitude({}) is unvalid'.format(altitude))
            return
        if not self.has_module('Baro'):
            logger.warn('Baro is closed')
            return
        CAlt = self.get_altitude(False)
        if CAlt is None:
            logger.error('Baro is not health')
            return
        if relative:
            TAlt = CAlt + altitude
        else:
            IAlt = self.ORB._HAL['InitAltitude']
            if IAlt is None:
                logger.error('InitAltitude is null')
                return
            TAlt = IAlt + altitude
        if TAlt < CAlt:
            logger.warn(
                'TAlt({}) is less than CAlt ({}).'.format(TAlt, CAlt))
            return
        self.control_FRU(THR=1)
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            CAlt = self.get_altitude(False)
            if CAlt is None or CAlt >= TAlt:
                break
            time.sleep(.1)
        self.brake()

    def down_metres(self, altitude, relative=True):
        if altitude <= 0:
            logger.warn('Altitude({}) is unvalid'.format(altitude))
            return
        if not self.has_module('Baro'):
            logger.warn('Baro is closed')
            return
        CAlt = self.get_altitude(False)
        if CAlt is None:
            logger.error('Barometre is not health')
            return

        TAlt = CAlt - altitude
        IAlt = self.ORB._HAL['InitAltitude']
        if IAlt is None:
            logger.error('InitAltitude is null')
            return
        if TAlt < IAlt + 1:
            logger.warn('TAltitude({}) is too low.'.format(TAlt - IAlt))
            return
        self.control_FRU(THR=-1)
        watcher = CancelWatcher()
        while not watcher.IsCancel():
            CAlt = self.get_altitude(False)
            if CAlt is None or CAlt <= TAlt:
                break
        self.brake()

    def yaw_left(self):
        self.control_FRU(RUD=-1)

    def yaw_right(self):
        self.control_FRU(RUD=1)

    def forward(self):
        logger.info('Forward...')
        self.control_FRU(ELE=1)

    def yaw_left_brake(self):
        logger.info('Yaw Left')
        self.control_FRU(RUD=-1)
        time.sleep(self.moveTime)
        self.brake()

    def yaw_right_brake(self):
        logger.info('Yaw Right')
        self.control_FRU(RUD=1)
        time.sleep(self.moveTime)
        self.brake()

    def forward_brake(self):
        logger.info('Forward')
        self.control_FRU(ELE=1)
        time.sleep(self.moveTime)
        self.brake()

    def backward_brake(self):
        logger.info('Backward')
        self.control_FRU(ELE=-1)
        time.sleep(self.moveTime)
        self.brake()

    def roll_left_brake(self):
        logger.info('Roll Left')
        self.control_FRU(AIL=-1)
        time.sleep(self.moveTime)
        self.brake()

    def roll_right_brake(self):
        logger.info('Roll Right')
        self.control_FRU(AIL=1)
        time.sleep(self.moveTime)
        self.brake()

    def up_brake(self):
        logger.info('Throttle Up')
        self.control_FRU(THR=1)
        time.sleep(self.moveTime)
        self.brake(1)

    def down_brake(self):
        logger.info('Throttle Down')
        self.control_FRU(THR=-1)
        time.sleep(self.moveTime)
        self.brake(1)

    def send_pwm(self, channels):
        # self._debug(channels)
        # self._debug(self.analysis_channels(channels))
        self.publish('ChannelsOutput', channels)

    def analysis_channels(self, channels):
        a = [x - y for x, y in zip(channels, self.subscribe('LoiterPWM'))]
        return [x * y for x, y in zip(a, self.Phase())]

    def isStop(self, heading, target, sign):
        diff = (360 + sign * (heading - target)) % 360
        return False if diff <= 180 and diff > 2 else True

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
            logger.debug('Turn left {}'.format(TurnAngle))
            self.yaw_left()
        else:
            is_cw = -1
            logger.debug('Turn right {}'.format(360 - TurnAngle))
            self.yaw_right()

        logger.debug("Target Angle: %d" % target_angle)
        while not watcher.IsCancel():
            current_yaw = self.get_heading()
            if current_yaw is None:
                break
            if self.isStop(current_yaw, target_angle, is_cw):
                break
            # logger.debug('{},{}'.format(current_yaw, target_angle))
        logger.debug("Before Angle:{}".format(self.get_heading()))
        self.brake()
        logger.debug("After  Angle:{}".format(self.get_heading()))

    def navigation(self, target):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = 1
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

            if not self.InAngle(angle, 90) or distance <= radius:
                logger.info("Reached Target!")
                break

            SAngle = int(math.degrees(math.asin(radius / distance)))

            self._debug('{} {} {}'.format(distance, angle, SAngle))

            if not self.InAngle(angle, max(SAngle, self.Epsilon)):
                self.brake()
                # location = self.get_location()
                # if location is None:
                #     break
                # angle = angle_heading_target(location, target, current_yaw)
                self.condition_yaw(angle)
            self.forward()
            time.sleep(frequency)
            # raw_input('next')
        self.brake()

    def navigation1(self, target):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = 1
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

            if not self.InAngle(angle, 90) or distance <= radius:
                logger.info("Reached Target Waypoint!")
                break
            SAngle = int(math.degrees(math.asin(radius / distance)))

            self._debug('{} {} {}'.format(distance, angle, SAngle))

            if self.InAngle(angle, max(SAngle, self.Epsilon)):
                self.control_FRU(ELE=1)
            else:
                if angle > SAngle and angle <= 90:
                    logger.debug('Roll Left')
                    self.control_FRU(AIL=-1, ELE=1)
                elif angle >= 270 and angle < 360 - SAngle:
                    logger.debug('Roll Right')
                    self.control_FRU(AIL=1, ELE=1)
                else:
                    self.brake()
                    self.condition_yaw(angle)
            time.sleep(frequency)
            # raw_input('next')
        self.brake()

    def navigation2(self, target):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = 1
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

            self._debug('{} {}'.format(distance, angle))

            if not self.InAngle(angle, 90) or distance <= radius:
                logger.info("Reached Target!")
                break

            self.forward()
            time.sleep(frequency)
            # raw_input('next')
        self.brake()

    def InAngle(self, angle, SAngle):
        if angle < 360 - SAngle and angle > SAngle:
            return False
        else:
            return True

    def Guided(self):
        target = self.get_target()
        if target is None:
            logger.warn("Target is None!")
            return
        self.publish('Mode', 'GUIDED')

        self.navigation(target)
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)

    def RTL(self):
        target = self.get_home()
        if target is None:
            logger.warn("Home is None!")
            return
        self.publish('Mode', 'RTL')

        self.navigation(target)
        # self.land()
        self.publish('Mode', 'STAB')

    def Route(self, info):
        self.wp.Route(info)
        # self.Auto()

    def Auto(self):
        if self.wp.isNull():
            logger.info('Waypoint is None')
            return
        self.publish('Mode', 'Auto')
        watcher = CancelWatcher()
        for point in self.wp.remain_wp():
            if watcher.IsCancel():
                self.publish('Mode', 'Loiter')
                return
            self.navigation(point)
            if not watcher.IsCancel():
                self.wp.add_number()

        self.publish('Mode', 'Loiter')
        self.wp.clear()

    def keycontrol(self):
        from keyboard_control import keyboard_event_wait, exe_cmd

        while True:
            command = keyboard_event_wait()
            if command == 'esc':
                break
            exe_cmd(self, command)
        time.sleep(.5)

    def Cancel(self):
        CancelWatcher.Cancel = True
        time.sleep(.1)
        self.brake()

if __name__ == "__main__":
    from uORB import uORB
    from library import Watcher

    ORB = uORB()
    Watcher()

    if has_module('Sbus'):
        # Initialize SBUS
        from sbus_receiver import Sbus_Receiver
        from sbus_sender import Sbus_Sender
        from tools import build_sbus

        com = build_sbus()
        sbus_receiver = Sbus_Receiver(ORB, com)
        sbus_receiver.start()

        while not ORB.state('Sbus'):
            time.sleep(.1)

        sbus_sender = Sbus_Sender(ORB, com)
        sbus_sender.start()

        while not ORB.state('Sender'):
            time.sleep(.1)
        print 'Sbus is OK'

    if has_module('Compass'):
        # Initialize Compass
        from compass_module import Compass
        compass = Compass(ORB)

        compass.start()
        while not ORB.state('Compass'):
            time.sleep(.1)
        print 'Compass is OK'

    if has_module('GPS'):
        # Initialize GPS
        from GPS_module import GPS
        gps = GPS(ORB)

        gps.start()
        while not ORB.state('GPS'):
            time.sleep(.1)
        print 'GPS is OK'

    if has_module('Baro'):
        # Initialize Barometre
        from Baro import Baro
        baro = Baro(ORB)

        baro.start()
        while not ORB.state('Baro'):
            time.sleep(.1)
        print 'Baro is OK'

    if has_module('IMU'):
        # Initialize IMU
        from IMU import IMU
        imu = IMU(ORB)

        imu.start()
        while not ORB.state('IMU'):
            time.sleep(.1)
        print 'IMU is OK'

    # Save FlightLog to SD card
    # ORB.start()

    # Initialize UAV
    vehicle = Vehicle(ORB)
    # vehicle.keycontrol()
    # Test
    import sys

    for t in commands:
        enter = raw_input(t + ' ???').strip()

        if enter == 'c' or enter == 'C':
            continue
        elif enter == 'b' or enter == 'B':
            break
        else:
            command = 'vehicle.' + t
            print 'Execute command ->', command
            try:
                eval(command)
            except Exception:
                info = sys.exc_info()
                print "{0}:{1}".format(*info)
                vehicle.Cancel()

    print 'Completed'
