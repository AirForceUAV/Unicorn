#!/usr/bin/evn python
# coding:utf-8

import time
from library import get_location_metres, isNum
from library import sin, cos, pressure2Alt
from waypoint import Waypoint


class Attribute(object):

    def __init__(self, ORB):
        self.ORB = ORB
        self._model = ORB._model['Model']
        from tools import logger
        self.logger = logger
        self._log('Drone Type:{}'.format(ORB._model['UAV']))
        # self._log('Drone model:{}'.format(ORB._model['Model']))
        self._log('MainController:{}'.format(ORB._model['MainController']))
        # Aileron :[No.ch, low ,mid, high ,var, sign, rate]
        self.AIL = ORB.channel('AIL')
        # Elevator:[No.ch, low ,mid, high ,var, sign, rate]
        self.ELE = ORB.channel('ELE')
        # Throttle:[No.ch, low ,mid, high ,var, sign, rate]
        self.THR = ORB.channel('THR')
        # Rudder  :[No.ch, low ,mid, high ,var, sign, rate]
        self.RUD = ORB.channel('RUD')
        # Mode :[No.ch , 0 , pwm]
        self.mode = ORB.channel('Mode')
        if self._model == 'HELI':
            # GYRO Rate :[No.ch , 0 , pwm]
            self.Rate = ORB.channel('Rate')
            # PITCH :[No.ch , 0 , pwm]
            self.PIT = ORB.channel('PIT')
        else:
            # Aux1 :[No.ch , 0 , pwm]
            self.Aux1 = ORB.channel('Aux1')
            # Aux2 :[No.ch , 0 , pwm]
            self.Aux2 = ORB.channel('Aux2')
        # Switch :[No.ch , 0 , pwm]
        self.Switch = ORB.channel('Switch')
        self.wp = Waypoint(ORB)
        self.update_home()
        self.init_altitude()

    def update_home(self):
        if not self.has_module('GPS'):
            return
        self._log('Waiting for home location')
        while not self.state('GPS'):
            time.sleep(.1)
        home = self.get_location()
        self.publish('HomeLocation', home)
        self._log('Home location :{}'.format(home))

    def init_altitude(self):
        if not self.has_module('Baro'):
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
        if self._model == 'HELI':
            phase[self.PIT[0]] = self.PIT[5]
        return phase

    def set_channels_mid(self):
        self._log('Catching Loiter PWM...')
        mid = self.subscribe('ChannelsInput')
        if mid is None:
            return
        self._log('Channels Mid:{}'.format(mid))
        self.publish('LoiterPWM', mid)
        self.AIL[2] = mid[self.AIL[0]]
        self.ELE[2] = mid[self.ELE[0]]
        self.THR[2] = mid[self.THR[0]]
        self.RUD[2] = mid[self.RUD[0]]
        self.mode[2] = mid[self.mode[0]]
        if self._model == 'HELI':
            self.Rate[2] = mid[self.Rate[0]]
            self.PIT[2] = mid[self.PIT[0]]
        else:
            self.Aux1[2] = mid[self.Aux1[0]]
            self.Aux2[2] = mid[self.Aux2[0]]
        self.Switch[2] = mid[self.Switch[0]]

    def set_gear(self, gear):
        if int(gear) in [1, 2, 3]:
            self.publish('Gear', int(gear))

    def set_target(self, dNorth, dEast, alt=-1000, relative=True):
        origin = self.get_location()
        if origin is None:
            return

        if not (isNum(dNorth) and isNum(dEast)):
            return
        target = get_location_metres(origin, dNorth, dEast)
        target.append(alt)
        self.publish('Target', target)
        self._log('Target is {}'.format(target))

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
        if not self.has_module('Compass'):
            self._warn('Compass is closed')
            return None
        if not self.state('Compass'):
            self._error('Compass is not health')
            return None
        return self.subscribe('Attitude')[0]

    def get_roll(self):
        if not self.has_module('Compass'):
            self._warn('Compass is closed')
            return None
        if not self.state('Compass'):
            self._error('Compass is not health')
            return None
        return self.subscribe('Attitude')[1]

    def get_heading(self):
        if not self.has_module('Compass'):
            self._warn('Compass is closed')
            return None
        if not self.state('Compass'):
            self._error('Compass is not health')
            return None
        return self.subscribe('Attitude')[2]

    def get_attitude(self):
        if not self.has_module('Compass'):
            self._warn('Compass is closed')
            return None
        if not self.state('Compass'):
            self._error('Compass is not health')
            return None
        return self.subscribe('Attitude')

    def get_home(self):
        return self.subscribe("HomeLocation")

    def get_location(self):
        if not self.has_module('Compass'):
            self._warn('Compass is closed')
            return None
        if not self.state('Compass'):
            self._error('Compass is not health')
            return None
        return self.subscribe('Location')

    def get_altitude(self, relative=False):
        return self.ORB.get_altitude(relative)

    def publish(self, topic, value):
        self.ORB.publish(topic, value)

    def subscribe(self, topic):
        return self.ORB.subscribe(topic)

    def has_module(self, module):
        return self.ORB.has_module(module)

    def state(self, module):
        return self.ORB.state(module)

    def _log(self, msg):
        self.logger.info(msg)

    def _debug(self, msg):
        self.logger.debug(msg)

    def _error(self, msg):
        self.logger.error(msg)

    def _warn(self, msg):
        self.logger.warn(msg)
    
    def _critical(self,msg):
        self.logger.critical(msg)
