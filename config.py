#!/usr/bin/evn python
# coding:utf-8

import json
import toml
from library import Singleton


class Config:
    __metaclass__ = Singleton

    def __init__(self):
        with open("config.yaml") as f:
            self.conf = toml.loads(f.read())

        self.version = self.conf['version']
        self.debug = self.conf['debug']
        self.drone, self.channels = self.init_drone()
        self.volume = self.get_volume()

        self.open_module = self.conf['open_module']
        self.commands = self.conf['commands']

        self.mqtt_socket = self.sock()
        self.client_id = self.conf['client_id']

        self.context_topic = str(self.conf['topic']['publish']['full'])
        self.control_topic = str(self.conf['topic']['publish']['semi'])
        self.full_auto_topic = str(self.conf['topic']['subscribe']['full'])
        self.semi_auto_topic = str(self.conf['topic']['subscribe']['semi'])
        self.keyboard_topic = str(self.conf['topic']['keyboard'])

        self.sbus_serial = self.get_sbus()
        self.compass_serial = self.conf['compass']['port']
        self.GPS_serial = self.conf['GPS']['port']
        self.debug = self.conf['debug']
        self.drone, self.channels = self.init_drone()
        self.volume = self.get_volume()

        self.open_module = self.conf['open_module']
        self.commands = self.conf['commands']

        self.mqtt_socket = self.sock()
        self.client_id = self.conf['client_id']

        self.context_topic = str(self.conf['topic']['publish']['full'])
        self.control_topic = str(self.conf['topic']['publish']['semi'])
        self.full_auto_topic = str(self.conf['topic']['subscribe']['full'])
        self.semi_auto_topic = str(self.conf['topic']['subscribe']['semi'])
        self.keyboard_topic = str(self.conf['topic']['keyboard'])

        self.sbus_serial = self.get_sbus()
        self.compass_serial = self.conf['compass']['port']
        self.GPS_serial = self.conf['GPS']['port']
        self.IMU_serial = self.conf['IMU']['port']

    def main_channel(self, ch):
        num = ch[0] - 1
        low = ch[1]
        mid = ch[2]
        hig = ch[3]
        var = hig - low
        sign = ch[4]
        rate = ch[5]
        return [num, low, mid, hig, var, sign, rate]

    def aux_channel(self, ch):
        return [ch[0] - 1] + ch[1:]

    def init_drone(self):
        drone = dict()
        chs = dict()

        system = "System{}".format(self.conf["SystemID"])
        UAV_config = self.conf[system]
        # print UAV_config
        drone['UAV'] = UAV_config['UAV']
        drone['Model'] = UAV_config['Model']
        drone['MainController'] = UAV_config['MainController']
        drone['Gear'] = UAV_config["Gear"]

        chs['AIL'] = self.main_channel(UAV_config['AIL'])
        chs['ELE'] = self.main_channel(UAV_config['ELE'])
        chs['THR'] = self.main_channel(UAV_config['THR'])
        chs['RUD'] = self.main_channel(UAV_config['RUD'])
        chs['Mode'] = self.aux_channel(UAV_config['Mode'])
        chs['Switch'] = self.aux_channel(UAV_config['Switch'])

        if drone['Model'] == 'HELI':
            chs['Rate'] = self.aux_channel(UAV_config['Rate'])
            chs['PIT'] = self.main_channel(UAV_config['PIT'])
        else:
            chs['Aux1'] = self.aux_channel(UAV_config['Aux1'])
            chs['Aux2'] = self.aux_channel(UAV_config['Aux2'])
        return drone, chs

    def has_module(self, module):
        return module in self.open_module

    def get_sbus(self):
        import serial
        _sbus = {}
        _sbus['port'] = self.conf['sbus']['port']
        _sbus['baudrate'] = self.conf['sbus']['baudrate']
        _sbus['parity'] = serial.PARITY_EVEN
        _sbus['stopbits'] = serial.STOPBITS_TWO
        _sbus['bytesize'] = serial.EIGHTBITS
        return _sbus

    def get_uart(self, sensor):
        return (self.conf[sensor][port], self.conf[sensor][baudrate])

    def sock(self):
        return (self.conf['MQTT']['host'], self.conf['MQTT']['port'])

    def get_volume(self):
        stroke = [[]] * 8
        # print volume
        for v in self.channels.itervalues():
            stroke[v[0]] = [v[1], v[2], v[3]]
        return stroke

    def __str__(self):
        return json.dumps(self.__dict__, indent=1)

config = Config()

if __name__ == '__main__':
    # print config
    # print 'Drone', config.drone
    print 'Channels', config.channels
    print 'channels volume', config.volume
    # print 'open module', config.open_module
    # print config.has_module('GPS')
    # print 'commands', config.commands
