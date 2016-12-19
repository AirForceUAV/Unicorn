#!/usr/bin/evn python
# coding:utf-8

from library import Singleton
import json
import time
from library import get_distance_metres
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
                        'Cloud': config.get_cloud()[0]
                        }
        self._HAL = {'Compass_State': False, 'Attitude': None,
                     'Baro_State': False, 'Pressure': 1013.25,
                     'InitPressure': 1023, 'Temperature': '-100',
                     'GPS_State': False, 'Location': None, 'NumStars': 0,
                     'HomeLocation': None, 'Target': None,
                     'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                     'RPM': 1600}
        self._sensor = FlightLog.sensors()

    def run(self):
        self.save_log()

    def publish(self, topic, value):
        self._HAL[topic] = value

    def subscribe(self, topic):
        return self._HAL[topic]

    def has_module(self, module):
        if self.module[module] > 0:
            return True
        else:
            return False

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

    def json_all_wp(self):
        Waypoints = self._HAL['Waypoint']
        if Waypoints is []:
            return []
        result = []
        for point in Waypoints:
            result.append('{}+{}'.format(point[0], point[1]))
        return ','.join(result)

    def pressure2Alt(self, hpa):
        # mba===hpa
        if hpa is None:
            return None
        tmp = hpa / 1013.25
        return round((1 - tmp**0.190284) * 145366.45 * 0.3048, 2)

    def get_altitude(self):
        init_pressure = self._HAL['InitPressure']
        cur_pressure = self._HAL['Pressure']
        if init_pressure is None or cur_pressure is None:
            return None
        alt = self.pressure2Alt(cur_pressure) - \
            self.pressure2Alt(init_pressure)
        return round(alt, 2)

    def list2str(self, arrs):
        result = []
        if arrs is not None:
            for arr in arrs:
                result.append(str(arr))
            return ','.join(result)
        else:
            return None

    def __str__(self):
        return json.dumps(self._HAL)

    def update_GPS(self):
        gps = self._sensor.gps
        gps.state = self._HAL['GPS_State']
        gps.num_stars = self._HAL['NumStars']
        self.update_location(gps.location, self._HAL['Location'])

    def update_compass(self):
        compass = self._sensor.compass
        compass.state = self._HAL['Compass_State']
        self.update_attitude(compass.attitude, self._HAL['Attitude'])

    def update_location(self, ProtoLocation, Locaiton):
        if Locaiton is None:
            Location = [0, 0, -1]

        ProtoLocation.latitude = Locaiton[0]
        ProtoLocation.longitude = Locaiton[1]
        if len(Locaiton) > 2:
            ProtoLocation.altitude = Locaiton[2]

    def update_attitude(self, ProtoAttitude, Attitude):
        if Attitude is None:
            Attitude = [-1, -1, -1]

        ProtoAttitude.pitch = Attitude[0]
        ProtoAttitude.roll = Attitude[1]
        ProtoAttitude.yaw = Attitude[2]

    def update_Baro(self):
        baro = self._sensor.baro
        baro.state = self._HAL['Baro_State']
        baro.Pressure = self._HAL['Pressure']
        baro.Temperature = self._HAL['Temperature']
        baro.Altitude = self.get_altitude()

    def update_waypoint(self):
        waypoint = self._sensor.waypoint
        ID = self._HAL['WaypointID']
        waypoint.ID = ID
        if ID < 0:
            return 0
        waypoints = self._HAL['Waypoint']
        for wp in waypoints:
            self.update_location(waypoint.point.add(), wp)

    def dataflash(self):
        # data = {'DistancFromHome': self.distance_from_home(),
        #         'DistanceToTarget': self.distance_to_target(),
        #         'Altitude': self.get_altitude()
        #         }
        # data.update(self._HAL)

        self.update_GPS()
        self.update_compass()
        self.update_Baro()
        self.update_location(self._sensor.home, self._HAL['HomeLocation'])
        self.update_location(self._sensor.target, self._HAL['Target'])
        self._sensor.DistanceToTarget = self.distance_to_target()
        self._sensor.DistanceFromHome = self.distance_from_home()
        self.update_waypoint()
        # return self._sensor
        return self._sensor.SerializeToString()
        # return json.dumps(data)

    def localtime(self):
        x = time.localtime(time.time())
        return time.strftime('%Y-%m-%d %H:%M:%S', x)

    def save_log(self):
        file_path = self.build_log()
        # print file_path
        with open(file_path, 'w+') as f:
            # f.write(self.dataflash() + '#')
            while True:
                f.write(self.dataflash() + '#')
                time.sleep(.5)

    def build_log(self):
        parentdir = os.path.expanduser('~') + '/UAVLog/'
        file_path = parentdir + self.localtime() + '.log'
        return file_path


if __name__ == "__main__":
    from waypoint import Waypoint
    ORB = uORB()
    ORB._HAL = {'Compass_State': True, 'Attitude': [-0.32, 0.01, 66],
                'Baro_State': True, 'Pressure': 1013.25,
                'InitPressure': 1023, 'Temperature': 26,
                'GPS_State': True, 'Location': [36.11127966305683, 116.2222, 100],
                'NumStars': 16,
                'HomeLocation': [36.1111, 116.2222],
                'Target': [36.1111, 116.22286716842115],
                'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                'RPM': 1600}
    # print ORB._module
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    print ORB._HAL
    print ORB.dataflash()
    ORB.save_log()
