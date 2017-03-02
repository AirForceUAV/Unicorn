#!/usr/bin/evn python
# coding:utf-8

import time
from library import *
from waypoint import Waypoint
from config import config
from tools import logger


class Attribute(object):

    def __init__(self, ORB):
        self.ORB = ORB
        logger.info('Drone Type:{}'.format(config.drone['UAV']))
        logger.info('MainController:{}'.format(config.drone['MainController']))
        # Aileron :[No.ch, low ,mid, high ,var, sign, rate]
        self.AIL = config.channels['AIL']
        # Elevator:[No.ch, low ,mid, high ,var, sign, rate]
        self.ELE = config.channels['ELE']
        # Throttle:[No.ch, low ,mid, high ,var, sign, rate]
        self.THR = config.channels['THR']
        # Rudder  :[No.ch, low ,mid, high ,var, sign, rate]
        self.RUD = config.channels['RUD']
        # Mode :[No.ch , 0 , pwm]
        self.mode = config.channels['Mode']
        if config.drone['Model'] == 'HELI':
            # GYRO Rate :[No.ch , 0 , pwm]
            self.Rate = config.channels['Rate']
            # PITCH :[No.ch , 0 , pwm]
            self.PIT = config.channels['PIT']
        else:
            # Aux1 :[No.ch , 0 , pwm]
            self.Aux1 = config.channels['Aux1']
            # Aux2 :[No.ch , 0 , pwm]
            self.Aux2 = config.channels['Aux2']
        # Switch :[No.ch , 0 , pwm]
        self.Switch = config.channels['Switch']
        self.wp = Waypoint(ORB)
        self.update_home()
        self.init_altitude()

    def update_home(self):
        if not config.has_module('GPS'):
            return
        logger.info('Waiting for home location')
        while not self.state('GPS'):
            time.sleep(.1)
        home = self.get_location()
        self.publish('HomeLocation', home)
        logger.info('Home location :{}'.format(home))

    def init_altitude(self):
        if not config.has_module('Baro'):
            logger.warn('Baro is closed')
            return
        init_pressure = self.subscribe('Pressure')
        self.publish('InitAltitude', pressure2Alt(init_pressure))

    def get_stars(self):
        return self.subscribe('NumStars')

    def download(self, index=0):
        location = self.get_location()
        if location is None:
            return
        # location=[39.11111,116.33333]
        self.wp.download(location, index)

    def Phase(self):
        phase = [0] * 8
        phase[self.AIL[0]] = self.AIL[5]
        phase[self.ELE[0]] = self.ELE[5]
        phase[self.THR[0]] = self.THR[5]
        phase[self.RUD[0]] = self.RUD[5]
        phase[self.mode[0]] = 1
        if drone['Model'] == 'HELI':
            phase[self.PIT[0]] = self.PIT[5]
        return phase

    def set_channels_mid(self):
        logger.info('Catching Loiter PWM...')
        mid = self.subscribe('ChannelsInput')
        if mid is None:
            logger.error('Sbus reveiver is not health')
            return
        logger.info('Channels Mid:{}'.format(mid))
        self.publish('LoiterPWM', mid)
        self.AIL[2] = mid[self.AIL[0]]
        self.ELE[2] = mid[self.ELE[0]]
        self.THR[2] = mid[self.THR[0]]
        self.RUD[2] = mid[self.RUD[0]]

        if drone['Model'] == 'HELI':
            self.Rate[2] = mid[self.Rate[0]]
            self.PIT[2] = mid[self.PIT[0]]

    def set_gear(self, Gear):
        if int(Gear) in [1, 2, 3]:
            self.publish('Gear', int(Gear) - 1)

    def set_target(self, dNorth, dEast, alt=-1000, relative=True):
        origin = self.get_location()
        if origin is None:
            return

        if not (isNum(dNorth) and isNum(dEast)):
            return
        target = get_location_metres(origin, dNorth, dEast)
        target.append(alt)
        self.publish('Target', target)
        logger.info('Target is {}'.format(target))

    def set_target_angle(self, distance, angle, alt=-1000, relative=True):
        if not isNum(distance) or not isNum(angle):
            return
        angle = (360 + angle) % 360
        dNorth = round(cos(angle) * distance, 2)
        dEast = round(sin(angle) * distance, 2)
        self.set_target(dNorth, dEast, alt)

    def get_target(self):
        return self.subscribe('Target')

    def get_pitch(self):
        if not config.has_module('Compass'):
            logger.warn('Compass is closed')
            return None
        if not self.state('Compass'):
            logger.error('Compass is not health')
            return None
        return self.subscribe('Attitude')[0]

    def get_roll(self):
        if not config.has_module('Compass'):
            logger.warn('Compass is closed')
            return None
        if not self.state('Compass'):
            logger.error('Compass is not health')
            return None
        return self.subscribe('Attitude')[1]

    def get_heading(self):
        if not config.has_module('Compass'):
            logger.warn('Compass is closed')
            return None
        if not self.state('Compass'):
            logger.error('Compass is not health')
            return None
        return self.subscribe('Attitude')[2]

    def get_attitude(self):
        if not config.has_module('Compass'):
            logger.warn('Compass is closed')
            return None
        if not self.state('Compass'):
            logger.error('Compass is not health')
            return None
        return self.subscribe('Attitude')

    def get_home(self):
        return self.subscribe("HomeLocation")

    def get_location(self):
        if not config.has_module('Compass'):
            logger.warn('Compass is closed')
            return None
        if not self.state('Compass'):
            logger.error('Compass is not health')
            return None
        return self.subscribe('Location')

    def get_altitude(self, relative=False):
        return self.ORB.get_altitude(relative)

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def state(self, module):
        return self.ORB.state(module)
