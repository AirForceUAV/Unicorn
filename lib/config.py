#!/usr/bin/evn python
# coding:utf-8

import json
import toml


class Config:

    def __init__(self):
        import os
        file_path = os.path.join('..', 'Toml', 'config.yaml')
        with open(file_path, 'r') as f:
            conf = toml.loads(f.read())
        # print conf
        self.version = conf['version']
        self.debug = conf['debug']

        self.drone, self.channels ,self.direction= self.init_drone(conf)
        self.volume = self.get_volume()

        self.lidar_mqtt(conf)
        self.keyboard_mqtt(conf)
        self.OA_RPC_Config(conf)

        self.sensors_serial(conf)
        self._open_module = conf['open_module']
        self.commands = conf['commands']

    def lidar_mqtt(self, conf):
        self.lidar_socket = self.lidar_sock(conf)
        self.client_id = conf['client_id']
        self.context_topic = str(conf['topic']['publish']['full'])
        self.control_topic = str(conf['topic']['publish']['semi'])
        self.full_auto_topic = str(conf['topic']['subscribe']['full'])
        self.semi_auto_topic = str(conf['topic']['subscribe']['semi'])

    def keyboard_mqtt(self, conf):
        self.KB_socket = self.keyboard_sock(conf)
        self.keyboard_topic = str(conf['topic']['keyboard'])

    def sensors_serial(self, conf):
        self.sbus_serial = self.get_sbus(conf)
        self.compass = conf['compass']
        self.GPS = conf['GPS']
        self.IMU = conf['IMU']

    def OA_RPC_Config(self, conf):
        OA_RPC = conf['RPC']['OA']
        self.OA_rpc_host = OA_RPC['host']
        self.OA_rpc_port = OA_RPC['port']

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

    def init_drone(self, conf):
        drone = dict()
        chs = dict()

        system = "System{}".format(conf["SystemID"])
        UAV_config = conf[system]
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
            chs['PIT'] = self.aux_channel(UAV_config['PIT'])
        else:
            chs['Aux1'] = self.aux_channel(UAV_config['Aux1'])
            chs['Aux2'] = self.aux_channel(UAV_config['Aux2'])
        direction=UAV_config['direction']
        return drone, chs,direction

    def has_module(self, module):
        return module in self._open_module

    def get_sbus(self, conf):
        import serial
        _sbus = {}
        _sbus['port'] = conf['sbus']['port']
        _sbus['baudrate'] = conf['sbus']['baudrate']
        _sbus['parity'] = serial.PARITY_EVEN
        _sbus['stopbits'] = serial.STOPBITS_TWO
        _sbus['bytesize'] = serial.EIGHTBITS
        return _sbus

    def lidar_sock(self, conf):
        lidar = conf['MQTT']['lidar']
        return (lidar['host'], lidar['port'])

    def keyboard_sock(self, conf):
        KB = conf['MQTT']['keyboard']
        return (KB['host'], KB['port'])

    def get_volume(self):
        stroke = [[]] * 8
        # print volume
        for v in self.channels.itervalues():
            stroke[v[0]] = [v[1], v[2], v[3]]
        return stroke

    def InitLoiter(self):
        channel = [0] * 8
        for v in self.channels.itervalues():
            index = v[0]
            channel[index] = v[2]
        return channel

    def __str__(self):
        return json.dumps(self.__dict__, indent=1)

    def show(self):
        pass

config = Config()

if __name__ == '__main__':
    pass
    # print config
    # print 'Drone', config.drone
    # print 'Channels', config.channels
    # # print 'channels volume', config.volume
    # print 'open module', config.open_module
    # print 'commands', config.commands
    # print config.has_module('GCS')
    print config.direction
