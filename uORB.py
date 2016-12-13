#!/usr/bin/evn python
# coding:utf-8

from library import Singleton
import json
from library import get_distance_metres
from config import config


class uORB(object):
    __metaclass__ = Singleton

    def __init__(self):
        self._module = {'Compass': config.get_compass()[0],
                        'GPS': config.get_GPS()[0],
                        'Baro': config.get_Baro()[0],
                        'MCU': config.get_MCU()[0],
                        'Lidar': config.get_lidar()[0],
                        'Cloud': config.get_cloud()[0]
                        }
        self._HAL = {'Compass_State': -1, 'Attitude': None,
                     'Baro_State': -1, 'Pressure': 1013.25,
                     'InitPressure': 1023,
                     'GPS_State': -1, 'Location': None, 'Num_stars': 0,
                     'HomeLocation': None, 'Target': None,
                     'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                     'RPM': 1600}

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
            return get_distance_metres(location, target)

    def distance_from_home(self):
        location = self._HAL['Location']
        target = self._HAL['HomeLocation']
        if location is None or target is None:
            return -1
        else:
            return get_distance_metres(location, target)

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

    def dataflash(self):
        data = {'DistancFromHome': self.distance_from_home(),
                'DistanceToTarget': self.distance_to_target(),
                'Altitude': self.get_altitude()
                }
        data.update(self._HAL)
        return json.dumps(data)

if __name__ == "__main__":
    from waypoint import Waypoint
    ORB = uORB()
    ORB._HAL = {'Compass_State': 1, 'Attitude': [-0.32, 0.01, 66],
                'Baro_State': 1, 'Pressure': 1013.25,
                'InitPressure': 1023,
                'GPS_State': 1, 'Location': [36.11127966305683, 116.2222],
                'Num_stars': 16,
                'HomeLocation': [36.1111, 116.2222],
                'Target': [36.1111, 116.22286716842115],
                'Mode': 'Loiter', 'Waypoint': [], 'WaypointID': -1,
                'RPM': 1600}
    print ORB._module
    wp = Waypoint(ORB)
    origin = [36.111111, 116.222222]
    wp.download(origin, 0)
    print ORB.dataflash()
