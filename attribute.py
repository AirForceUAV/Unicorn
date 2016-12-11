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
        self.target = None                  # target location -- [lat,lon,alt]
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
        self.mode_name = 'Loiter'
        # 8 channels PWM:[0,CH1,CH2,CH3,CH4,CH5,CH6,CH7,CH8]
        self.channels = self.init_channels()
        self.channels_mid = self.init_channels_mid()
        self.wp = Waypoint()

        self.home_location = None
        self.init_alt = None
        if config.get_GPS()[0] > 0:
            self._log('Waiting for home location')
            while True:
                home = self.get_location()
                num_stars = self.get_stars()
                if home is not None:
                    self.home_location = home
                    break

            self._log('Home location :{}'.format(self.home_location))
        if config.get_Baro()[0] > 0:
            self.init_alt = round(self.convert2Alt(
                self.ORB.subscribe('Pressure')), 2)
            print 'init_alt is ', self.init_alt

    def get_stars(self):
        if config.get_GPS()[0] > 0:
            return self.ORB.subscribe('Num_Stars')
        else:
            return 0

    def download(self, index=0):
        if not config.get_GPS()[0] > 0:
            self._log("GPS is closed")
            return -1
        location = self.get_location()
        # loc=[39.11111,116.33333]
        if location is None:
            self._log('GPS is not health')
            return -1

        self.wp.download(location, index)

    def json_all_wp(self):
        if self.wp.all_wp() == []:
            return None
        result = []
        for point in self.wp.all_wp():
            result.append('{}+{}'.format(point[0], point[1]))
        return ','.join(result)

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
        if self._frame is 'HELI':
            channels[self.PIT[0]] = self.PIT_curve(channels[self.THR[0]])
        return channels

    def init_channels_mid(self):
        channels = [0, 0, 0, 0, 0, 0, 0, 0]
        channels[self.AIL[0]] = self.AIL[2]
        channels[self.ELE[0]] = self.ELE[2]
        channels[self.THR[0]] = self.THR[2]
        channels[self.RUD[0]] = self.RUD[2]
        channels[self.mode[0]] = self.mode[1]
        if self._frame is 'HELI':
            channels[self.PIT[0]] = self.PIT_curve(channels[self.THR[0]])
        return channels

    def update_PIT(self, THR_PWM):
        if self._frame is "HELI":
            self.channels[self.PIT[0]] = self.PIT_curve(THR_PWM)

    def PIT_curve(self, pwm):
        per = 100 * (pwm - self.THR[1]) / (self.THR[3] - self.THR[1])
        PIT_PWM = int(((0.0022 * per * per - 0.85 * per + 63) / 63.0)
                      * (self.PIT[3] - self.PIT[2])) + self.PIT[2]
        return PIT_PWM

    def set_channels_mid(self):
        self._log('Catching Loiter PWM...')
        if not self.config.get_MCU()[0] > 0 or self.mcu is None:
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
        if not self.ORB.has_topic('GPS'):
            return 0
        origin = self.get_location()
        if origin is None:
            self._log('GPS is Error')
            return -1
        if not isNum(dNorth) or not isNum(dEast):
            self._log('dNorth , dEast are unvalid')
            return -1
        if alt is None:
            self._log('Set to Current Altitude')
        self.target = get_location_metres(origin, dNorth, dEast)

    def set_target_angle(self, distance, angle):
        rad = math.radians(angle)
        dNorth = round(math.cos(rad), 1)
        dEast = round(math.sin(rad), 1)
        self.set_target(dNorth, dEast)

    def get_target(self):
        return self.target

    def get_heading(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.ORB.subscribe('Compass_State'):
            return None
        else:
            return self.ORB.subscribe('Attitude')[2]

    def get_pitch(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.ORB.subscribe('Compass_State'):
            return None
        else:
            return self.ORB.subscribe('Attitude')[0]

    def get_roll(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.ORB.subscribe('Compass_State'):
            return None
        else:
            return self.ORB.subscribe('Attitude')[1]

    def get_attitude(self):
        if not config.get_compass()[0] > 0:
            return None

        if self.ORB.subscribe('Compass_State'):
            return None
        else:
            return self.ORB.subscribe('Attitude')

    def get_mode(self):
        return self.mode_name

    def distance_from_home(self):
        if not config.get_GPS()[0] > 0:
            return -1
        location = self.get_location()
        home = self.get_home()
        if location is None or home is None:
            return -1
        else:
            return get_distance_metres(location, home)

    def distance_to_target(self):
        if not config.get_GPS()[0] > 0:
            return -1
        location = self.get_location()
        target = self.get_target()
        if location is None or target is None:
            return -1
        else:
            return get_distance_metres(location, target)

    def set_home(self):
        if not config.get_GPS()[0] > 0:
            return -1
        self.home_location = self.get_location()

    def get_home(self):
        return self.home_location

    def get_location(self):
        if not config.get_GPS()[0] > 0:
            return None
        if self.ORB.subscribe('GPS_State') is -1:
            return None
        else:
            loc = self.ORB.subscribe('Location')
            return loc

    def get_alt(self):
        hpa = self.ORB.subscribe('Pressure')
        if self.init_alt is None or hpa is None:
            return None
        else:
            return self.convert2Alt(hpa) - self.init_alt

    def convert2Alt(self, hpa):
        mbar = hpa / 1013.25
        return round((1 - mbar**0.190284) * 145366.45 * 0.3048, 2)

    def FlightLog(self):
        log = {}
        log["id"] = time.time()
        if config.get_GPS()[0] > 0:
            log["HomeLocation"] = self.list2str(
                self.get_home())           # lat,lon,alt
            log["LocationGlobal"] = self.list2str(
                self.get_location())    # lat,lon,alt
            log["DistanceFromHome"] = self.distance_from_home()  # distance
            log["DistanceToTarget"] = self.distance_to_target()  # distance
            log['Target'] = self.list2str(self.get_target())  # lat,lon
        else:
            log["HomeLocation"] = "{},{},{}".format(36.11111, 116.22222, 0.0)
            log["LocationGlobal"] = "{},{},{}".format(36.01234, 116.12375, 0.0)
            log["DistanceFromHome"] = 0.0
            log["DistanceToTarget"] = 0.0
            log['Target'] = "{},{},{}".format(36.01234, 116.12375, 0.0)
        log["Battery"] = '{},{},{}'.format(
            12, 2.5, 100)       # [Voltage,Current,Capacity]
        log["Velocity"] = "{},{},{}".format(0.2, 0.1, 0.5)      # [x,y,z]
        log["EKF"] = 1
        log["Groundspeed"] = 2.0   # speed
        log["Airspeed"] = 3.0     # speed
        log["Mode"] = self.get_mode()  # mode
        log["IMU"] = -1
        log["TimeStamp"] = int(time.time())
        log['Gear'] = self.get_gear()  # Gear
        log['CurrentChannels'] = self.list2str(self.channels)   # ch1~ch8
        log['LoiterChannels'] = self.list2str(self.channels_mid)  # ch1~ch8
        log['CurrentWpNumber'] = self.wp._number
        log['AllWp'] = self.json_all_wp()
        log['RPM'] = 1600    # RPM

        return json.dumps(log)

    def list2str(self, arrs):
        result = []
        if arrs is not None:
            for arr in arrs:
                result.append(str(arr))
            return ','.join(result)
        else:
            return None

    def _log(self, msg):
        print msg
