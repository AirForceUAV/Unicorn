#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
import math
from attribute import Attribute
from AF_ML.Curve import THR2PIT
from lib.config import config
from lib.tools import CancelWatcher, Singleton
from lib.science import get_distance_metres, angle_heading_target, angle_diff
from lib.logger import logger


class Vehicle(Attribute):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        super(Vehicle, self).__init__(ORB)
        self.moveTime = 2
        self.Epsilon = 20
        self.radius = 5
        self.frequence = 1
        self.prepre_state = [0]
        self.pre_state = [0]
        self._state = [0]

    def brake(self, braketime=0.5):
        self.send_pwm(self.subscribe('LoiterPWM'))
        time.sleep(braketime)


    def _brake(self):
        self.send_pwm(self.subscribe('LoiterPWM'))

    def control_stick(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=2):
        channels = [0] * 8
        channels[self.AIL[0]] = self.AIL[2 + AIL * self.AIL[5]]
        channels[self.ELE[0]] = self.ELE[2 + ELE * self.ELE[5]]
        channels[self.THR[0]] = self.THR[2 + THR * self.THR[5]]
        channels[self.RUD[0]] = self.RUD[2 + RUD * self.RUD[5]]
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def control_FRU(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=2):
        channels = [0] * 8
        channels[self.AIL[0]] = self.movement(self.AIL, AIL)
        channels[self.ELE[0]] = self.movement(self.ELE, ELE)
        channels[self.THR[0]] = self.movement(self.THR, THR)
        channels[self.RUD[0]] = self.movement2(self.RUD, RUD)
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def control_percent(self, AIL=0, ELE=0, THR=0, RUD=0, Mode=2):
        channels = [0] * 8
        channels[self.AIL[0]] = self.movement3(self.AIL, AIL)
        channels[self.ELE[0]] = self.movement3(self.ELE, ELE)
        channels[self.THR[0]] = self.control_THR(THR)
        channels[self.RUD[0]] = self.movement3(self.RUD, RUD)
        channels[self.mode[0]] = self.mode[Mode]
        self._construct_channel(channels)
        self.send_pwm(channels)

    def _construct_channel(self, channels):
        if config.drone['Model'] == 'HELI':
            channels[self.Rate[0]] = self.Rate[2]
            channels[self.PIT[0]] = THR2PIT(channels[self.THR[0]])
            # channels[self.PIT[0]] = self.PIT[0]
        else:
            channels[self.Aux1[0]] = self.Aux1[2]
            channels[self.Aux2[0]] = self.Aux2[2]
        channels[self.Switch[0]] = self.Switch[2]

    def movement(self, channel, sign=1):
        # sign in [-1,0,1]. By Gear
        index = self.subscribe('Gear')
        rate = config.drone['Gear'][index] / 100.0
        index = 2 + channel[5] * sign
        section = abs(channel[2] - channel[index])
        variation = int(channel[5] * section * rate)
        return channel[2] + sign * variation

    def movement2(self, channel, sign=1):
        # sign in [-1,0,1].  By TOML
        rate = channel[6] / 100.0
        index = 2 + channel[5] * sign
        section = abs(channel[2] - channel[index])
        variation = int(channel[5] * section * rate)
        return channel[2] + sign * variation

    def movement3(self, channel, percent=0):
        # -100 <= percent <= 100. By Percent
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
        watcher = CancelWatcher()
        if begin <= end:
            while begin < end and not watcher.IsCancel():
                self.control_percent(THR=begin)
                begin += 1
                time.sleep(0.05)
        else:
            while begin > end and not watcher.IsCancel():
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

        self.control_stick(-1, -1, -1, 1)
        # time.sleep(2)
        # self.disarm()

    def disarm(self):
        logger.info('DisArmed ...')
        self.control_stick(THR=-1)
        time.sleep(.5)
        self.control_stick(1, -1, -1, 1)
        time.sleep(2)
        self.control_stick(THR=-1)

    def takeoff(self, alt=5):
        watcher = CancelWatcher()
        if not self.state('Baro'):
            return
        logger.info('Takeoff to {} m'.format(alt))

        self.escalate(0, 60)

        while not watcher.IsCancel():
            try:
                currentAlt = self.get_altitude(True)
            except AssertionError, e:
                logger.error(e)
                break
            if currentAlt > alt * 0.95:
                logger.info('Reached Altitude :{}'.format(currentAlt))
                break
            time.sleep(.01)

        self.brake()

    def land(self):
        logger.info('Landing...')

    def up_metres(self, altitude, relative=True):
        if altitude <= 0:
            logger.warn('Altitude({}) is invalid'.format(altitude))
            return
        try:
            CAlt = self.get_altitude(False)

            if relative:
                TAlt = CAlt + altitude
            else:
                init_alt = self.ORB.get_init_alt()
                TAlt = init_alt + altitude
            if TAlt < CAlt:
                logger.warn(
                    'TAlt({}) is less than CAlt ({}).'.format(TAlt, CAlt))
                return
            self.GradualTHR(0, 60)
            watcher = CancelWatcher()
            while not watcher.IsCancel():
                CAlt = self.get_altitude(False)
                if CAlt >= TAlt:
                    break
                time.sleep(.1)
            self.brake()
        except AssertionError, e:
            logger.error(e)

    def down_metres(self, altitude):
        watcher = CancelWatcher()
        if altitude <= 0:
            logger.warn('Altitude({}) is invalid'.format(altitude))
            return
        try:
            CurAlt = self.get_altitude(False)
            TarAlt = CAlt - altitude
            InitAlt = self.ORB.get_init_alt()
            if TAlt < IAlt + 1:
                logger.warn('TAltitude({}) is too low.'.format(TAlt - IAlt))
                return
            self.control_FRU(THR=-1)
            watcher = CancelWatcher()
            while not watcher.IsCancel():
                CAlt = self.get_altitude(False)
                if CAlt <= TAlt:
                    break
            self.brake()
        except AssertionError, e:
            logger.error(e)

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
        # time.sleep(self.moveTime)
        # self.brake()

    def yaw_right_brake(self):
        logger.info('Yaw Right')
        self.control_FRU(RUD=1)
        # time.sleep(self.moveTime)
        # self.brake()

    def forward_brake(self):
        logger.info('Forward')
        self.control_FRU(ELE=1)
        # time.sleep(self.moveTime)
        # self.brake()

    def backward_brake(self):
        logger.info('Backward')
        self.control_FRU(ELE=-1)
        # time.sleep(self.moveTime)
        # self.brake()

    def roll_left_brake(self):
        logger.info('Roll Left')
        self.control_FRU(AIL=-1)
        # time.sleep(self.moveTime)
        # self.brake()

    def roll_right_brake(self):
        logger.info('Roll Right')
        self.control_FRU(AIL=1)
        # time.sleep(self.moveTime)
        # self.brake()

    def up_brake(self):
        logger.info('Throttle Up')
        self.control_FRU(THR=1)
        # time.sleep(self.moveTime)
        # self.brake(1)

    def down_brake(self):
        logger.info('Throttle Down')
        self.control_FRU(THR=-1)
        # time.sleep(self.moveTime)
        # self.brake(1)

    def send_pwm(self, channels):
        # logger.debug(channels)
        # logger.debug(self.analysis_channels(channels))
        self.publish('ChannelsOutput', channels)

    def analysis_channels(self, channels):
        a = [x - y for x, y in zip(channels, self.subscribe('LoiterPWM'))]
        return [x * y for x, y in zip(a, self.Phase())]

    def isStop(self, heading, target, sign):
        diff = angle_diff(heading, target, sign)

        return False if diff <= 180 and diff > 2 else True

    def condition_yaw(self, heading):
        """
        0<=heading<360 (anti-clockwise)
        """
        if config.debug:
            logger.warn('condition_yaw() ...')
            return
        watcher = CancelWatcher()
        assert heading >= 0 and heading < 360, 'Param-heading {} is invalid'.format(
            heading)

        CYaw = self.get_heading()
        target_angle = angle_diff(CYaw, heading)
        TurnAngle = angle_diff(CYaw, target_angle)

        if TurnAngle >= 0 and TurnAngle <= 180:
            is_cw = 1
            logger.debug('Turn left {}'.format(TurnAngle))
            self.yaw_left()
        else:
            is_cw = -1
            logger.debug('Turn right {}'.format(360 - TurnAngle))
            self.yaw_right()

        logger.debug("Current Angle:{} Target Angle:{}".format(
            CYaw, target_angle))
        while not watcher.IsCancel():
            CYaw = self.get_heading()
            # logger.debug("Current Angle:{} Target Angle:{}".format(
            #     CYaw, target_angle))
            if self.isStop(CYaw, target_angle, is_cw):
                break
            time.sleep(.01)
        self.brake()
        logger.debug("Fact Angle: %d" % self.get_heading())

    def _navigation(self):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = self.frequence
        try:
            target = self.get_target()
            CLocation = self.get_location()
            CYaw = self.get_heading()

            init_angle = angle_heading_target(CLocation, target, CYaw)
            self.condition_yaw(init_angle)

            while not watcher.IsCancel():
                CLocation = self.get_location()
                CYaw = self.get_heading()
                distance = get_distance_metres(CLocation, target)
                angle = angle_heading_target(CLocation, target, CYaw)

                if not self.InAngle(angle, 90) or distance <= radius:
                    logger.info("Reached Target!")
                    self.brake()
                    return True

                EAngle = int(math.degrees(math.asin(radius / distance)))

                logger.debug('{} {} {}'.format(distance, angle, EAngle))

                if not self.InAngle(angle, max(EAngle, self.Epsilon)):
                    self.brake()
                    self.condition_yaw(angle)
                self.forward()
                time.sleep(frequency)
                # raw_input('next')
        except AssertionError, e:
            self.brake()
            logger.error(e)
            return False

    def navigation(self):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = self.frequence
        try:
            target = self.get_target()
            CLocation = self.get_location()
            CYaw = self.get_heading()

            init_angle = angle_heading_target(CLocation, target, CYaw)
            self.condition_yaw(init_angle)

            while not watcher.IsCancel():
                CLocation = self.get_location()
                CYaw = self.get_heading()

                distance = get_distance_metres(CLocation, target)
                angle = angle_heading_target(CLocation, target, CYaw)

                if not self.InAngle(angle, 90) or distance <= radius:
                    # if distance <= radius:
                    logger.info("Reached Target Waypoint!")
                    self.brake()
                    return True
                EAngle = int(math.degrees(math.asin(radius / distance)))

                logger.debug('{} {} {}'.format(distance, angle, EAngle))

                if self.InAngle(angle, max(EAngle, self.Epsilon)):
                    self.control_FRU(ELE=1)
                else:
                    if angle > EAngle and angle <= 90:
                        logger.debug('Roll Left')
                        self.control_FRU(AIL=-1, ELE=1)
                    elif angle >= 270 and angle < 360 - EAngle:
                        logger.debug('Roll Right')
                        self.control_FRU(AIL=1, ELE=1)
                    else:
                        self.brake()
                        self.condition_yaw(angle)
                time.sleep(frequency)
        except AssertionError, e:
            self.brake()
            logger.error(e)
            return False

    def __navigation(self):
        watcher = CancelWatcher()
        radius = self.radius
        frequency = 1
        try:
            target = self.get_target()
            CLocation = self.get_location()
            CYaw = self.get_heading()

            init_angle = angle_heading_target(CLocation, target, CYaw)
            self.condition_yaw(init_angle)

            while not watcher.IsCancel():
                CLocation = self.get_location()
                CYaw = self.get_heading()
                distance = get_distance_metres(CLocation, target)
                angle = angle_heading_target(CLocation, target, CYaw)

                logger.debug('{} {}'.format(distance, angle))

                if not self.InAngle(angle, 90) or distance <= radius:
                    logger.info("Reached Target!")
                    self.brake()
                    return True
                self.forward()
                time.sleep(frequency)
        except AssertionError, e:
            self.brake()
            logger.error(e)
            return False

    def InAngle(self, angle, EAngle):
        if angle < 360 - EAngle and angle > EAngle:
            return False
        else:
            return True

    def Guided(self):
        logger.info('GUIDED...')
        flag = True
        try:
            target = self.get_target()
        except AssertionError, e:
            logger.error(e)
            return False
        self.publish('Mode', 'GUIDED')
        result = self.navigation()
        if not result:
            logger.error("Navigation except exit")
            flag = False
        self._finally()
        return flag

    def RTL(self):
        logger.info('RTL...')
        flag = True
        try:
            home = self.get_home()
        except AssertionError, e:
            logger.error(e)
            return False
        self.publish('Mode', 'RTL')
        self.publish('Target', home)
        result = self.navigation()
        if not result:
            logger.error("Navigation except exit")
            flag = False
        # self.land()
        self._finally()
        return flag

    def Route(self, info):
        self.wp.Route(info)
        # self.Auto()

    def _Route(self, points):
        self.wp._Route(points)

    def Auto(self):
        logger.info('Auto...')
        flag = True
        if self.wp.isNull():
            logger.error('Waypoint is None')
            return False
        self.publish('Mode', 'Auto')
        watcher = CancelWatcher()
        for point in self.wp.points:
            if watcher.IsCancel():
                logger.warn('Cancel Auto')
                flag = False
                break
            self.publish('Target', point)
            result = self.navigation()
            if not result:
                logger.error("Navigation except exit")
                flag = False
                break
            self.wp.add_number()

        self.Auto_finally()
        return flag

    def _finally(self):
        self.publish('Mode', 'Loiter')
        self.publish('Target', None)
        self.brake()

    def Auto_finally(self):
        self._finally()
        self.wp.clear()

    def Cancel(self):
        CancelWatcher.Cancel = True
        time.sleep(.1)
        self.brake()


def init_sensors(ORB):
    if config.has_module('Sbus'):
        # Initialize SBUS
        from AF_Sbus.sender import sbus_start
        sbus_start(ORB)

    if config.has_module('Compass'):
        # Initialize Compass
        from AF_Sensors.compass import compass_start
        compass_start(ORB)

    if config.has_module('GPS'):
        # Initialize GPS
        from AF_Sensors.GPS import GPS_start
        GPS_start(ORB)

    if config.has_module('Baro'):
        # Initialize Barometre
        from AF_Sensors.Baro import Baro_start
        Baro_start(ORB)

    if config.has_module('IMU'):
        # Initialize IMU
        from AF_Sensors.IMU import IMU_start
        IMU_start(ORB)


def init_vehicle(ORB):

    init_sensors(ORB)
    '''Save FlightLog to SD card'''
    # ORB.start()

    vehicle = Vehicle(ORB)
    return vehicle

if __name__ == "__main__":
    from lib.tools import Watcher
    from AF_uORB.uORB import uORB

    ORB = uORB()
    Watcher()

    '''Initialize UAV'''
    vehicle = init_vehicle(ORB)
    time.sleep(2)
    # vehicle.set_target(100,0)

    commands = {'stop': 'brake()',
                'arm': 'arm()',
                'mid': 'set_channels_mid()',
                'gear': 'set_gear(2)',
                'yawl': 'yaw_left_brake()',
                'yawr': 'yaw_right_brake()',
                'rolll': 'roll_left_brake()',
                'rollr': 'roll_right_brake()',
                'forward': 'forward_brake()',
                'back': 'backward_brake()',
                'up': 'up_brake()',
                'down': 'down_brake()',
                'conl': 'condition_yaw(30)',
                'conr': 'condition_yaw(300)',
                'target': 'set_target(100,0)',
                'guided': 'Guided()',
                'download': 'download()',
                'auto': 'Auto()',
                'disarm': 'disarm()',
                'rtl': 'RTL()',
                'esc': 'esc',
                's': 'brake()',
                }

    while True:
        enter = raw_input('Input:').strip()
        cmd = commands.get(enter)
        if cmd == None:
            print 'input is error {}'.format(enter)
            continue
        elif cmd == 'esc':
            break

        command = 'vehicle.' + cmd
        print 'Execute command ->', command
        try:
            eval(command)
        except Exception, e:
            info = sys.exc_info()
            print "{0}:{1}".format(*info)
            vehicle.Cancel()

    # for c in config.commands:
    #     enter = raw_input(c + ' ?').strip()

    #     if enter == 'c':
    #         continue
    #     elif enter == 'b':
    #         break
    #     else:
    #         command = 'vehicle.' + c
    #         print 'Execute command ->', command
    #         try:
    #             eval(command)
    #         except Exception, e:
    #             info = sys.exc_info()
    #             print "{0}:{1}".format(*info)
    #             vehicle.Cancel()

    print 'Completed'
