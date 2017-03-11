#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import json
import time
import threading
import protobuf.FlightLog_pb2 as FlightLog
from lib.science import get_distance_metres, pressure2Alt
from lib.config import config
from lib.tools import Singleton


class uORB(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self):
        super(uORB, self).__init__(name='uORB')
        LoiterPWM=self.InitLoiter()
        
        if config.debug:
            from test import FlightLog
            self._HAL = FlightLog(LoiterPWM)
        else:
            self._HAL = {'Compass_State': False,
                        'Attitude': None,
                        'Baro_State': False,
                        'Pressure': None,
                        'Temperature': None,
                        'Gear': 0,
                        'GPS_State': False,
                        'Location': None,
                        'NumStars': 0,
                        'HomeLocation': None,
                        'Target': None,
                        'Mode': 'STAB',
                        'Waypoint': [],
                        'WaypointID': -1,
                        'RPM': 1600,
                        'Sbus_State':False,
                        'ChannelsOutput': None,
                        'ChannelsInput': None,
                        'LoiterPWM': LoiterPWM,
                        'InitAltitude': None,
                        'IMU_State': False,
                        'WaypointType': None,
                        'ACC': None, 'GYR': None, 'MAG': None, 'EUL': None,
                        'QUA': None}

    # def run(self):
    #     self.save_log()

    def publish(self, topic, value):
        if self._HAL[topic] != value:
            self._HAL[topic] = value

    def subscribe(self, topic):
        return self._HAL[topic]

    def state(self, module):
        return self._HAL[module + '_State']

    def InitLoiter(self):
        channel = [0] * 8
        for v in config.channels.itervalues():
            index = v[0]
            channel[index] = v[2]
        return channel
        
    def distance_to_target(self):
        try:
            location = self.get_location()
            target = self.get_target()
        except AssertionError,e:
            return -1
        distance = get_distance_metres(location, target)
        return distance

    def distance_from_home(self):
        try:
            location = self.get_location()
            home = self.get_home()
        except AssertionError,e:
            return -1

        distance = get_distance_metres(location, home)
        return distance

    def get_attitude(self):
        assert self.state('Compass'),'Compass is not health'
        return self._HAL['Attitude']

    def get_heading(self):
        return self.get_attitude()[2]

    def get_location(self):
        assert self.state('GPS'),'GPS is not health'
        location = self._HAL['Location']
        return location

    def get_home(self):
        home = self._HAL['HomeLocation']
        assert home != None,'Home is Null'
        return home

    def get_target(self):
        target = self._HAL['Target']
        assert target != None,'Target is Null'
        return target

    def get_altitude(self, relative=False):
        CPressure = self.get_pressure()
        init_alt = self.get_init_alt()
        CurAlt= pressure2Alt(CPressure)
        if relative:
            alt = CurAlt - init_alt
        else:
            alt = CurAlt
        return alt

    def get_init_alt(self):
        initAlt = self._HAL['InitAltitude']
        assert initAlt != None,'Init Altitude is Null'
        return initAlt

    def get_pressure(self):
        assert self.state('Baro') == True,'Barometre is not health'
        return self._HAL['Pressure']

    def update_location(self, ProtoLocation, Locaiton):
        if Locaiton is None:
            return
        ProtoLocation.latitude = Locaiton[0]
        ProtoLocation.longitude = Locaiton[1]
        if len(Locaiton) == 3:
            ProtoLocation.altitude = Locaiton[2]

    def update_attitude(self, ProtoAttitude, Attitude):
        if Attitude is None:
            return
        ProtoAttitude.pitch,ProtoAttitude.roll,ProtoAttitude.yaw = Attitude

    def update_GPS(self):
        gps = self._sensor.gps
        gps.state = self.state('GPS')
        gps.num_stars = self._HAL['NumStars']
        try:
            location = self.get_location()
            self.update_location(gps.location, location)
        except AssertionError,e:
            pass       

    def update_compass(self):
        compass = self._sensor.compass
        compass.state = self.state('Compass')
        try:
            attitude = self.get_attitude()
            self.update_attitude(compass.attitude, attitude)
        except AssertionError,e:
            pass

    def update_Baro(self):
        baro = self._sensor.baro
        baro.state = self.state('Baro')
        try:
            baro.Pressure = self.get_pressure()      
            baro.Temperature = self._HAL['Temperature']  
            baro.Altitude = self.get_altitude(True)
        except AssertionError,e:
            pass

    def update_waypoint(self):
        waypoint = self._sensor.waypoint
        index = self._HAL['WaypointID']

        waypoint.index = index
        waypoint.type = Type
        if index < 0:
            return
        waypoints = self._HAL['Waypoint']
        id = 0
        for wp in waypoints:
            point = waypoint.point.add()
            point.ID = id
            self.update_location(point.location, wp)
            id += 1

    def update_channelsInput(self):
        c = self._sensor.ChannelsInput
        channels = self._HAL['ChannelsInput']
        if channels is None:
            return
        c.ch1, c.ch2, c.ch3, c.ch4, c.ch5, c.ch6, c.ch7, c.ch8 = channels

    def update_channelsOutput(self):
        c = self._sensor.ChannelsOutput
        channels = self._HAL['ChannelsOutput']
        if channels is None:
            return
        c.ch1, c.ch2, c.ch3, c.ch4, c.ch5, c.ch6, c.ch7, c.ch8 = channels

    def update_loiterPWM(self):
        c = self._sensor.LoiterPWM
        channels = self._HAL['LoiterPWM']
        if channels is None:
            return
        c.ch1, c.ch2, c.ch3, c.ch4, c.ch5, c.ch6, c.ch7, c.ch8 = channels

    def dataflash(self):
        return self.log_proto()

    def log_proto(self):
        self._sensor = FlightLog.sensors()
        self._sensor.timestamp = time.time()
        self.update_GPS()
        self.update_compass()
        self.update_Baro()
        self.update_location(self._sensor.home, self._HAL['HomeLocation'])
        self.update_location(self._sensor.target, self._HAL['Target'])
        self._sensor.Gear = self._HAL['Gear'] + 1
        self._sensor.DistanceToTarget = self.distance_to_target()
        self._sensor.DistanceFromHome = self.distance_from_home()
        # self.update_waypoint()
        self.update_channelsInput()
        self.update_channelsOutput()
        self.update_loiterPWM()
        self._sensor.Mode = self._HAL['Mode']
        # return self._sensor
        return self._sensor.SerializeToString()

    def __str__(self):
        return json.dumps(self._HAL, indent=1)


if __name__ == "__main__":
    from AF_Copter.waypoint import Waypoint
    from lib.tools import Watcher

    ORB = uORB()
    Watcher()
    # print ORB

    # wp = Waypoint(ORB)
    # origin = [36.111111, 116.222222]
    # wp.download(origin, 0)
    # wp.add_number()
    print ORB.dataflash()

    # b = FlightLog.sensors()
    # b.ParseFromString(ORB.dataflash())
    # print b
