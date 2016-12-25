#!/usr/bin/evn python
# coding:utf-8

import time
from library import get_location_metres, isNum
from library import sin, cos, pressure2Alt
from waypoint import Waypoint


class Attribute(object):

    def __init__(self, mcu, ORB):
        self.mcu = mcu
        self.ORB = ORB
        self._model = ORB._model['Model']
        self._log('Drone :{}'.format(ORB._model['UAV']))
        self._log('Drone model:{}'.format(ORB._model['Model']))
        self._log('MainController:{}'.format(ORB._model['MainController']))
        # Aileron :[No.ch, low ,mid, high ,var, sign, rate]
        self.AIL = ORB.channel('AIL')
        # Elevator:[No.ch, low ,mid, high ,var, sign, rate]
        self.ELE = ORB.channel('ELE')
        # Throttle:[No.ch, low ,mid, high ,var, sign, rate]
        self.THR = ORB.channel('THR')
        # Rudder  :[No.ch, low ,mid, high ,var, sign, rate]
        self.RUD = ORB.channel('RUD')

        self.mode = ORB.channel('Mode')
        if ORB._model['Model'] == 'HELI':
            self.Rate = ORB.channel('Rate')
            self.PIT = ORB.channel('PIT')
        else:
            self.Aux1 = ORB.channel('Aux1')
            self.Aux2 = ORB.channel('Aux2')
            # 8 channels PWM:[CH1~CH8]
        self.Switch = ORB.channel('Switch')
        self.wp = Waypoint(ORB)
        self.update_home()
        self.init_altitude()

    def update_home(self):
        if not self.has_module('GPS'):
            return
        self._log('Waiting for home location')
        while not self.state('GPS'):
            time.sleep(.5)
        home = self.get_location()
        self.publish('HomeLocation', home)
        self._log('Home location :{}'.format(home))

    def init_altitude(self):
        if self.has_module('Baro'):
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
        phase = [1] * 8
        phase[self.AIL[0]] = self.AIL[5]
        phase[self.ELE[0]] = self.ELE[5]
        phase[self.THR[0]] = self.THR[5]
        phase[self.RUD[0]] = self.RUD[5]
        return phase

    def THR2PIT(self, THR):
        per = 100 * (THR - self.THR[1]) / self.THR[4]
        p1 = ((0.0022 * per * per - 0.85 * per + 63) / 63.0) * self.PIT[4]
        PIT_PWM = int(p1) + self.PIT[1]
        return PIT_PWM

    def set_channels_mid(self):
        a = time.time()
        self._log('Catching Loiter PWM...')
        if not self.ORB.has_module('MCU'):
            print 'Warning:MCU is closed'
            return
        mid = self.mcu.read_channels()
        if mid is None:
            self._log('Error:Co-MCU is not health')
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
        b = time.time()
        print b - a

    def set_gear(self, gear):
        if int(gear) in [1, 2, 3]:
            self.publish('Gear', int(gear))

    def set_target(self, dNorth, dEast, alt=-1):
        origin = self.get_location()
        if origin is None:
            return

        if not (isNum(dNorth) and isNum(dEast)):
            return
        target = get_location_metres(origin, dNorth, dEast).append(alt)
        self.publish('Target', target)

    def set_target_angle(self, distance, angle, alt=-1):
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
            self._log('Warning:Compass is closed')
            return None
        if not self.state('Compass'):
            self._log('Error:Compass is not health')
            return None
        return self.subscribe('Attitude')[0]

    def get_roll(self):
        if not self.has_module('Compass'):
            self._log('Warning:Compass is closed')
            return None
        if not self.state('Compass'):
            self._log('Error:Compass is not health')
            return None
        return self.subscribe('Attitude')[1]

    def get_heading(self):
        if not self.has_module('Compass'):
            self._log('Warning:Compass is closed')
            return None
        if not self.state('Compass'):
            self._log('Error:Compass is not health')
            return None
        return self.subscribe('Attitude')[2]

    def get_attitude(self):
        if not self.has_module('Compass'):
            self._log('Warning:Compass is closed')
            return None
        if not self.state('Compass'):
            self._log('Error:Compass is not health')
            return None
        return self.subscribe('Attitude')

    def get_home(self):
        return self.subscribe("HomeLocation")

    def get_location(self):
        if not self.has_module('GPS'):
            self._log('Warning:GPS is closed')
            return None
        if not self.state('GPS'):
            self._log('Error:GPS is not health')
            return None
        return self.subscribe('Location')

    def get_alt(self, relative=True):
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
        print ">>>", msg
