#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import time
from lib.science import *
from waypoint import Waypoint
from lib.config import config
from lib.logger import logger


class Attribute(object):

    def __init__(self, ORB):
        self.ORB = ORB
        logger.info('Drone Type:{}'.format(config.drone['UAV']))
        logger.info('MainController:{}'.format(config.drone['MainController']))
        self._model = config.drone['Model']
        # Aileron :[No.ch, low ,mid, high ,var, sign, rate]
        self.AIL = config.channels['AIL']
        # Elevator:[No.ch, low ,mid, high ,var, sign, rate]
        self.ELE = config.channels['ELE']
        # Throttle:[No.ch, low ,mid, high ,var, sign, rate]
        self.THR = config.channels['THR']
        # Rudder  :[No.ch, low ,mid, high ,var, sign, rate]
        self.RUD = config.channels['RUD']
        # Mode :[No.ch , low , Loiter, high]
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
        logger.info('Waiting for home location')
        try:
            home = self.get_location()
            self.publish('HomeLocation', home)
            logger.info('Home location :{}'.format(home))
        except AssertionError, e:
            logger.error(e)

    def init_altitude(self):
        logger.info('Waiting for init altitude')
        try:
            init_alt = self.get_altitude(False)
            self.publish('InitAltitude', init_alt)
            logger.info('Init Altitude :{}'.format(init_alt))
        except AssertionError, e:
            logger.error(e)

    def get_stars(self):
        return self.subscribe('NumStars')

    def isArmed(self):
        return self.subscribe('Armed')

    def _armed(self):
        self.publish('Armed', True)

    def _disarmed(self):
        self.publish('Armed', False)

    def download(self, index=1):
        try:
            location = self.get_location()
            self.wp.download(location, index)
        except AssertionError, e:
            logger.error(e)

    def Phase(self):
        phase = [0] * 8
        phase[self.AIL[0]] = self.AIL[5]
        phase[self.ELE[0]] = self.ELE[5]
        phase[self.THR[0]] = self.THR[5]
        phase[self.RUD[0]] = self.RUD[5]
        # phase[self.mode[0]] = 1
        if config.drone['Model'] == 'HELI':
            phase[self.PIT[0]] = self.PIT[4]
        return phase

    def set_channels_mid(self):
        logger.info('Catching Loiter PWM...')
        if self.state('Sbus'):
            mid = self.subscribe('ChannelsInput')
        else:
            logger.error('Sbus receiver is not health')
            return False
        logger.info('Channels Mid:{}'.format(mid))
        self.publish('LoiterPWM', mid)
        self.AIL[2] = mid[self.AIL[0]]
        self.ELE[2] = mid[self.ELE[0]]
        self.THR[2] = mid[self.THR[0]]
        self.RUD[2] = mid[self.RUD[0]]

        if self._model == 'HELI':
            self.Rate[2] = mid[self.Rate[0]]
            self.PIT[2] = mid[self.PIT[0]]
        return True
    
    def get_gear(self):
        return self.subscribe('Gear')
        
    def set_gear(self, Gear):
        if int(Gear) in [1, 2, 3]:
            self.publish('Gear', int(Gear) - 1)
            return True
        else:
            return False

    def set_target(self, dNorth, dEast):
        try:
            origin = self.get_location()
            target = get_location_metres(origin, dNorth, dEast)
            self.publish('Target', target)
            logger.info('Target is {}'.format(target))
            return True
        except AssertionError, e:
            logger.error(e)
            return False

    def set_target_angle(self, distance, angle):
        angle = (360 + angle) % 360
        dNorth = round(cos(angle) * distance, 2)
        dEast = round(sin(angle) * distance, 2)
        self.set_target(dNorth, dEast)

    def get_target(self):
        return self.ORB.get_target()

    def get_home(self):
        return self.ORB.get_home()

    def get_location(self):
        return self.ORB.get_location()

    def get_heading(self):
        return self.ORB.get_heading()

    def get_altitude(self, relative=False):
        return self.ORB.get_altitude(relative)

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def state(self, module):
        return self.ORB.state(module)

    def has_module(self, module):
        return config.has_module(module)

if __name__ == "__main__":
    from AF_uORB.uORB import uORB

    ORB = uORB()

    try:
        print attr.get_location()
        print attr.get_home()
        print attr.get_target()
        print attr.get_heading()
    except AssertionError, e:
        logger.error(e)
