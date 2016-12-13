#!/usr/bin/evn python
# coding:utf-8

from config import config
import time
import json
import math
from library import list_assign, get_location_metres, get_distance_metres, isNum
from waypoint import Waypoint


class Attribute(object):

    def __init__(self, mcu=None, ORB=None):
        self._log('Vehicle Type:{}'.format(config.get_vehicle()))
        self._log('Flight Controller:{}'.format(config.get_FC()))
        self.mcu = mcu
        self.ORB = ORB
        self._frame = config.get_frame()
        # Aileron :[ch number,low PWM ,mid PWM,high PWM ,variation
        # PWM,dir,rate]
        self.AIL = config.get_AIL()
        # Elevator:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.ELE = config.get_ELE()
        # Throttle:[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.THR = config.get_THR()
        # Rudder  :[ch number,low PWM ,mid PWM,high PWM ,var,dir,rate]
        self.RUD = config.get_RUD()
        self.mode = config.get_mode()     # Mode    :[ch number,Loiter PWM]
        self.PIT = config.get_PIT()      # Pitch   :[chnumber, exit?]
        self.gear = config.get_gear()
        self.MD = config.get_MD()
        self.BD = config.get_BD()
        self.DD = config.get_DD()
        # 8 channels PWM:[CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels = self.init_channels()
        self.channels_mid = self.init_channels_mid()
        self.wp = Waypoint(ORB)
        # self.update_home()
        self.init_pressure()

    def update_home(self):
        if config.get_GPS()[0] > 0:
            self._log('Waiting for home location')
            while True:
                home = self.get_location()
                num_stars = self.get_stars()
                if home is not None and num_stars > 9:
                    self.publish('HomeLocation', home)
                    break

            self._log('Home location :{}'.format(home))

    def init_pressure(self):
        if config.get_Baro()[0] > 0:
            init_pressure = self.subscribe('Pressure')
            self.publish('InitPressure', init_pressure)
            print 'init_pressure is ', init_pressure

    def get_stars(self):
        if config.get_GPS()[0] > 0:
            return self.subscribe('Num_Stars')
        else:
            return 0

    def download(self, index=0):
        if not config.get_GPS()[0] > 0:
            self._log("GPS is closed")
            return 0
        location = self.get_location()
        # loc=[39.11111,116.33333]
        if location is None:
            self._log('GPS is not health')
            return -1

        self.wp.download(location, index)

    def init_channels(self):
        channels = [0, 0, 0, 0, 0, 0, 0, 0]
        channels[self.AIL[0]] = self.AIL[2]
        channels[self.ELE[0]] = self.ELE[2]
        if self.THR[5] > 0:
            channels[self.THR[0]] = self.THR[1]
        else:
            channels[self.THR[0]] = self.THR[3]
        channels[self.RUD[0]] = self.RUD[2]
        channels[self.mode[0]] = self.mode[1]
        self.update_PIT(channels[self.THR[0]])
        return channels

    def init_channels_mid(self):
        channels = [0, 0, 0, 0, 0, 0, 0, 0]
        channels[self.AIL[0]] = self.AIL[2]
        channels[self.ELE[0]] = self.ELE[2]
        channels[self.THR[0]] = self.THR[2]
        channels[self.RUD[0]] = self.RUD[2]
        channels[self.mode[0]] = self.mode[1]
        self.update_PIT(self.THR[2])
        return channels

    def update_PIT(self, THR_PWM):
        if self._frame is "HELI":
            self.channels[self.PIT[0]] = self.PIT_curve(THR_PWM)

    def PIT_curve(self, pwm):
        per = 100 * (pwm - self.THR[1]) / self.THR[4]
        PIT_PWM = int(((0.0022 * per * per - 0.85 * per + 63) / 63.0)
                      * (self.PIT[3] - self.PIT[2])) + self.PIT[2]
        return PIT_PWM

    def set_channels_mid(self):
        self._log('Catching Loiter PWM...')
        if not config.get_MCU()[0] > 0 or self.mcu is None:
            return 0
        mid = self.mcu.read_channels()
        if mid is None:
            self._log('Co-MCU is dead')
            return -1
        self._log('Channels Mid:{}'.format(mid))
        list_assign(self.channels, mid)
        list_assign(self.channels_mid, mid)
        self.AIL[2] = mid[self.AIL[0]]
        self.ELE[2] = mid[self.ELE[0]]
        self.THR[2] = mid[self.THR[0]]
        self.RUD[2] = mid[self.RUD[0]]

    def set_gear(self, gear):
        if int(gear) in [1, 2, 3]:
            self.gear[0] = int(gear)
        else:
            self._log('Gear is unvalid')

    def get_gear(self):
        return int(self.gear[0])

    def set_target(self, dNorth, dEast, alt=None):
        if not config.get_GPS()[0] > 0:
            return 0
        origin = self.get_location()
        if origin is None:
            self._log('GPS is not health')
            return -1
        if not isNum(dNorth) or not isNum(dEast):
            return -1
        if alt is None:
            alt = self.get_alt()
        target = get_location_metres(origin, dNorth, dEast).append(alt)
        self.publish('Target', target)

    def set_target_angle(self, distance, angle, alt=None):
        if not isNum(distance) or not isNum(angle):
            return -1
        angle = (360 + angle) % 360
        rad = math.radians(angle)
        dNorth = round(math.cos(rad) * distance, 2)
        dEast = round(math.sin(rad) * distance, 2)
        self.set_target(dNorth, dEast, alt)

    def get_target(self):
        return self.subscribe('Target')

    def get_heading(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.subscribe('Compass_State') is -1:
            return None
        else:
            return self.subscribe('Attitude')[2]

    def get_pitch(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.subscribe('Compass_State') is -1:
            return None
        else:
            return self.subscribe('Attitude')[0]

    def get_roll(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.subscribe('Compass_State') is -1:
            return None
        else:
            return self.subscribe('Attitude')[1]

    def get_attitude(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.subscribe('Compass_State') is -1:
            return None
        else:
            return self.subscribe('Attitude')

    def get_home(self):
        return self.subscribe("HomeLocation")

    def get_location(self):
        if not config.get_GPS()[0] > 0 or self.subscribe('GPS_State') is -1:
            return None
        else:
            loc = self.subscribe('Location')
            return loc

    def get_alt(self):
        return self.ORB.get_altitude()

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def _log(self, msg):
        print msg
