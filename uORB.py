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
from sbus import SBUS
from tools import build_log


class uORB(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self):
        super(uORB, self).__init__(name='uORB')
        self.sbus = SBUS()
        model = ['UAV', 'Model', 'MainController']
        self._model = {}
        for m in model:
            self._model[m] = config._config[m]

        module = ['Sbus', 'Compass', 'GPS', 'IMU', 'Baro', 'Lidar', 'Cloud']
        # self._module = {}
        # for m in module:
        #     self._module[m] = False
        self._module = {x: False for x in module}
        from tools import open_module
        self.open(*open_module)

        channel = ['AIL', 'ELE', 'THR', 'RUD', 'Mode', 'Switch']
        if self._model['Model'] == 'HELI':
            channel += ['Rate', 'PIT']
        else:
            channel += ['Aux1', 'Aux2']
        # self._channel = {}
        # for c in channel:
        #     self._channel[c] = config._config[c]
        self._channel = {x: config._config[x] for x in channel}
        self._volume = [0] * 8
        for k, v in self._channel.iteritems():
            self._volume[v[0]] = (v[1], v[2], v[3])

        Gear = config._config['Gear']
        self._Gear = Gear[1:]

        self._HAL = {'Compass_State': False, 'Attitude': None,
                     'Baro_State': False, 'Pressure': None,
                     'Temperature': None, 'Gear': Gear[0],
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
        self._sensor = FlightLog.sensors()

    def run(self):
        self.save_log()

    def publish(self, topic, value):
        self._HAL[topic] = value

    def subscribe(self, topic):
        return self._HAL[topic]

    def state(self, module):
        return self._HAL[module + '_State']

    def channel(self, channel):
        return self._channel[channel]

    def has_module(self, module):
        return self._module[module]

    def open(self, *module):
        for x in module:
            self._module[x] = True

    def close(self, *module):
        for x in module:
            self._module[x] = False

    def InitLoiter(self):
        channels = [0] * 8
        for ch, ch_val in self._channel.iteritems():
            channels[ch_val[0]] = ch_val[3] if ch == 'Mode' else ch_val[2]
        return channels

    def InitChannels(self):
        channels = [0] * 8
        if self._model['Model'] == 'HELI':
            for ch, ch_val in self._channel.iteritems():
                if ch == 'Mode':
                    channels[ch_val[0]] = ch_val[3]
                elif ch == 'THR' or ch == 'PIT':
                    channels[ch_val[0]] = ch_val[2 - ch_val[5]]
                else:
                    channels[ch_val[0]] = ch_val[2]
        else:
            for ch, ch_val in self._channel.iteritems():
                if ch == 'Mode':
                    channels[ch_val[0]] = ch_val[3]
                elif ch == 'THR':
                    channels[ch_val[0]] = ch_val[2 - ch_val[5]]
                else:
                    channels[ch_val[0]] = ch_val[2]
        return channels

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
        ID = self._HAL['WaypointID']
        if ID is -1:
            return ""
        points = self._HAL['Waypoint']
        result = ['+'.join(map(str, p)) for p in points]
        return ','.join(result)

    def get_altitude(self, relative=False):
        init_altitude = self._HAL['InitAltitude']
        cur_pressure = self._HAL['Pressure']
        if init_altitude is None or cur_pressure is None:
            return None
        alt = pressure2Alt(
            cur_pressure) - init_altitude if relative else pressure2Alt(cur_pressure)
        return round(alt, 2)

    def list2str(self, rawlist):
        if rawlist is None:
            return ""
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
            baro.Altitude = self.get_altitude(True)

    def update_waypoint(self):
        waypoint = self._sensor.waypoint
        index = self._HAL['WaypointID']
        Type = self._HAL['WaypointType']

        waypoint.index = index
        if Type is not None:
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
        # return self.log_json()

    def log_proto(self):
        self._sensor = FlightLog.sensors()
        self._sensor.timestamp = time.time()
        self.update_GPS()
        self.update_compass()
        self.update_Baro()
        self.update_location(self._sensor.home, self._HAL['HomeLocation'])
        self.update_location(self._sensor.target, self._HAL['Target'])
        self._sensor.Gear = self._HAL['Gear']
        self._sensor.DistanceToTarget = self.distance_to_target()
        self._sensor.DistanceFromHome = self.distance_from_home()
        self.update_waypoint()
        self.update_channelsInput()
        self.update_channelsOutput()
        self.update_loiterPWM()
        self._sensor.Mode = self._HAL['Mode']
        # return self._sensor
        return self._sensor.SerializeToString()

    def log_json(self):
        log = {}
        log['Timestamp'] = time.time()
        log['DistanceToTarget'] = self.distance_to_target()
        log['DistanceFromHome'] = self.distance_from_home()
        log.update(self._HAL)
        # log['Waypoint'] = self.json_points()
        # log['Target'] = self.list2str(self._HAL['Target'])
        return json.dumps(log)

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
    ORB = uORB()
    from tools import protobuf, commands

    ORB._HAL = protobuf
    print ORB._model
    print [k for k, v in ORB._module.iteritems() if v]
    print '--------Channel-------------\n{}' .format(ORB._channel)
    print '--------servo volume--------\n{}'.format(ORB._volume)
    print 'commands:', json.dumps(commands, indent=1)
    # wp = Waypoint(ORB)
    # origin = [36.111111, 116.222222]
    # wp.download(origin, 0)
    # wp.add_number()
    print ORB.dataflash()

    # b = FlightLog.sensors()
    # b.ParseFromString(ORB.dataflash())
    # print b

    """Save FlightLog to SB card"""
    # print('Save Log')
    # Watcher()
    # ORB.start()
