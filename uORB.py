#!/usr/bin/evn python
# coding:utf-8

from library import Singleton
import json
import time
from library import get_distance_metres, pressure2Alt
from config import config
import FlightLog_pb2 as FlightLog
import os
import threading


class uORB(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self):
        super(uORB, self).__init__(name='uORB')
        self._module = {'Compass': config.get_compass()[0],
                        'GPS': config.get_GPS()[0],
                        'Baro': config.get_Baro()[0],
                        'MCU': config.get_MCU()[0],
                        'Lidar': config.get_lidar()[0],
                        'Cloud': config.get_cloud()[0],
                        'UAV': config.get_vehicle(),
                        'Type': config.get_frame(),
                        'FC': config.get_FC()
                        }
        self._HAL = {'Compass_State': False, 'Attitude': None,
                     'Baro_State': False, 'Pressure': None,
                     'Temperature': None,
                     'GPS_State': False, 'Location': None, 'NumStars': 0,
                     'HomeLocation': None, 'Target': None,
                     'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                     'RPM': 1600, 'ChannelsOutput': [0] * 8,
                     'ChannelsInput': [0] * 8, 'LoiterPWM': [0] * 8,
                     'InitAltitude': None,'IMU_State':False,
                     'ACC':None,'GYR':None,'MAG':None,'EUL':None,
                     'QUA':None}
        self._sensor = FlightLog.sensors()

    def run(self):
        self.save_log()

    def publish(self, topic, value):
        self._HAL[topic] = value

    def subscribe(self, topic):
        return self._HAL[topic]

    def state(self, module):
        return self._HAL[module + '_State']

    def has_module(self, module):
        return True if self._module[module] > 0 else False

    def open(self, *module):
        for x in module:
            self._module[x] = 1

    def close(self, *module):
        for x in module:
            self._module[module] = -1

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

    def json_points(self):
        points = self._HAL['Waypoint']
        ID = self._HAL['WaypointID']
        if ID is -1:
            return ""
        result = ['+'.join(map(str, p)) for p in points]
        return ','.join(result)

    def get_altitude(self, relative=True):
        init_altitude = self._HAL['InitAltitude']
        cur_pressure = self._HAL['Pressure']
        if init_altitude is None or cur_pressure is None:
            return None
        alt = pressure2Alt(
            cur_pressure) - init_altitude if relative else pressure2Alt(cur_pressure)
        return round(alt, 2)

    def list2str(self, rawlist):
        if rawlist is None:
            return None
        else:
            return ','.join(map(str, rawlist))

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
            baro.Altitude = self.get_altitude()

    def update_waypoint(self):
        waypoint = self._sensor.waypoint
        index = self._HAL['WaypointID']
        waypoint.index = index
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
        pass

    def update_channelsOutput(self):
        c = self._sensor.ChannelsOutput
        c.ch1, c.ch2, c.ch3, c.ch4, c.ch5, c.ch6, c.ch7, c.ch8 = self._HAL[
            'ChannelsOutput']

    def update_loiterPWM(self):
        c = self._sensor.LoiterPWM
        c.ch1, c.ch2, c.ch3, c.ch4, c.ch5, c.ch6, c.ch7, c.ch8 = self._HAL[
            'LoiterPWM']

    def dataflash(self):
        return self.log_proto()
        # return self.log_json()

    def log_proto(self):
        self._sensor.timestamp = time.time()
        self.update_GPS()
        self.update_compass()
        self.update_Baro()
        self.update_location(self._sensor.home, self._HAL['HomeLocation'])
        self.update_location(self._sensor.target, self._HAL['Target'])
        self._sensor.DistanceToTarget = self.distance_to_target()
        self._sensor.DistanceFromHome = self.distance_from_home()
        self.update_waypoint()
        self.update_channelsInput()
        self.update_channelsOutput()
        self.update_loiterPWM()
        return self._sensor

    def log_json(self):
        log = {}
        log['Timestamp'] = time.time()
        log['DistanceToTarget'] = self.distance_to_target()
        log['DistanceFromHome'] = self.distance_from_home()
        log.update(self._HAL)
        # log['Waypoint'] = self.json_points()
        # log['Target'] = self.list2str(self._HAL['Target'])
        return json.dumps(log)

    def localtime(self):
        x = time.localtime(time.time())
        return time.strftime('%Y-%m-%d %H:%M:%S', x)

    def save_log(self):
        file_path = self.build_log()
        # print file_path
        with open(file_path, 'w+') as f:
            while True:
                f.write(self.log_proto().SerializeToString() + '##')
                time.sleep(.5)

    def build_log(self):
        log_name = self.localtime() + '.log'
        file_path = os.path.join(os.path.expanduser('~'), 'UAVLog', log_name)
        return file_path

    def __str__(self):
        return json.dumps(self._HAL, indent=1)


if __name__ == "__main__":
    from waypoint import Waypoint
    from library import Watcher
    ORB = uORB()
    # print ORB.build_log()
    ORB.open("Compass")
    ORB._HAL = {'Compass_State': True, 'Attitude': [-0.32, 0.01, 66],
                'Baro_State': True, 'Pressure': 1013.25,
                'Temperature': 26, 'ChannelsInput': [10] * 8,
                'GPS_State': True, 'Location': [36.11127966305683, 116.2222, 100],
                'NumStars': 16, 'ChannelsOutput': [10] * 8,
                'HomeLocation': [36.1111, 116.2222],
                'Target': [36.1111, 116.22286716842115],
                'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                'RPM': 1600, 'InitAltitude': -80.81,
                'LoiterPWM': [10] * 8}
    print json.dumps(ORB._module, indent=1)
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    # print ORB._HAL
    print ORB.dataflash()
    # Watcher()
    # ORB.start()
    # ORB.join()
