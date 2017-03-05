#!/usr/bin/evn python
# coding:utf-8


import json
import time
import threading
import FlightLog_pb2 as FlightLog
from library import get_distance_metres, pressure2Alt
from config import config
# from tools import build_log
from library import Singleton


class uORB(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self):
        super(uORB, self).__init__(name='uORB')
        self.model = config.drone['Model']

        self._HAL = {'Compass_State': False, 'Attitude': None,
                     'Baro_State': False, 'Pressure': None,
                     'Temperature': None, 'Gear': 1,
                     'GPS_State': False, 'Location': None, 'NumStars': 0,
                     'HomeLocation': None, 'Target': None,
                     'Mode': 'STAB', 'Waypoint': [], 'WaypointID': -1,
                     'RPM': 1600, 'Sbus_State': False, 'ChannelsOutput': None,
                     'ChannelsInput': self.InitChannels(),
                     'LoiterPWM': self.InitLoiter(),
                     'InitAltitude': None, 'IMU_State': False,
                     'Sender_State': False, 'WaypointType': None,
                     'ACC': None, 'GYR': None, 'MAG': None, 'EUL': None,
                     'QUA': None}

    def run(self):
        self.save_log()

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

    def InitChannels(self):
        channels = [0] * 8
        return None

    def distance_to_target(self):
        location = self._HAL['Location']
        target = self._HAL['Target']
        if location is None or target is None:
            return -1
        else:
            distance = get_distance_metres(location, target)
            return round(distance, 2)

    def distance_from_home(self):
        location = self._HAL['Location']
        target = self._HAL['HomeLocation']
        if location is None or target is None:
            return -1
        else:
            distance = get_distance_metres(location, target)
            return round(distance, 2)

    def get_altitude(self, relative=False):
        init_altitude = self._HAL['InitAltitude']
        cur_pressure = self._HAL['Pressure']
        if init_altitude is None or cur_pressure is None:
            return None
        alt = pressure2Alt(
            cur_pressure) - init_altitude if relative else pressure2Alt(cur_pressure)
        return round(alt, 2)

    def update_location(self, ProtoLocation, Locaiton):
        if Locaiton is None:
            return
        ProtoLocation.latitude = Locaiton[0]
        ProtoLocation.longitude = Locaiton[1]
        if len(Locaiton) > 2:
            ProtoLocation.altitude = Locaiton[2]

    def update_attitude(self, ProtoAttitude, Attitude):
        if Attitude is None:
            return
        ProtoAttitude.pitch = Attitude[0]
        ProtoAttitude.roll = Attitude[1]
        ProtoAttitude.yaw = Attitude[2]

    def update_GPS(self):
        gps = self._sensor.gps
        gps.state = self._HAL['GPS_State']
        gps.num_stars = self._HAL['NumStars']
        if self.state("GPS"):
            self.update_location(gps.location, self._HAL['Location'])

    def update_compass(self):
        compass = self._sensor.compass
        compass.state = self._HAL['Compass_State']
        if self.state("Compass"):
            self.update_attitude(compass.attitude, self._HAL['Attitude'])

    def update_Baro(self):
        baro = self._sensor.baro
        baro.state = self._HAL['Baro_State']
        if self.state('Baro'):
            baro.Pressure = self._HAL['Pressure']
            baro.Temperature = self._HAL['Temperature']
            baro.Altitude = self.get_altitude(True)

    def update_waypoint(self):
        Type = self._HAL['WaypointType']

        if Type != 'Download':
            return
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

    def save_log(self):
        file_path = build_log(self._model['Model'], 'HAL')
        # print file_path
        with open(file_path, 'w+') as f:
            while True:
                f.write(self.log_proto() + '##')
                time.sleep(.5)

    def __str__(self):
        return json.dumps(self._HAL, indent=1)


if __name__ == "__main__":
    from waypoint import Waypoint
    from library import Watcher
    from test_data import protobuf
    ORB = uORB()

    print ORB._HAL
    # wp = Waypoint(ORB)
    # origin = [36.111111, 116.222222]
    # wp.download(origin, 0)
    # wp.add_number()
    # print ORB.dataflash()

    # b = FlightLog.sensors()
    # b.ParseFromString(ORB.dataflash())
    # print b

    """Save FlightLog to SB card"""
    # print('Save Log')
    # Watcher()
    # ORB.start()
    # print ORB
