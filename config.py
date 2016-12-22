#!/usr/bin/evn python
# coding:utf-8

from library import Singleton, element
import json


class Config(object):
    __metaclass__ = Singleton

    def __init__(self):
        file_name = 'Vehicle.xml'
        root_node = element(file_name)
        ID = int(root_node.get('ID'))
        self._root = root_node[ID]
        self._config = {}
        self._config['vehicle'] = self._root.get('vehicle')
        self._config['frame'] = self._root.get('frame')
        self._config['FC'] = self._root.get('FlightController')
        self._config['cloud'] = self.loadXML(0, 3)  # [open?,ip,port]
        self._config['AIL'] = self.ch(1)
        self._config['ELE'] = self.ch(2)
        self._config['THR'] = self.ch(3)
        self._config['RUD'] = self.ch(4)
        self._config['PIT'] = self.loadPIT()
        self._config['mode'] = [self.get_node(6, 1) - 1, self.get_node(6, 2)]
        self._config['MCU'] = self.loadXML(7, 4)  # [open?,port,baudrate]
        self._config['GPS'] = self.loadXML(8, 4)  # [open?,port,baudrate]
        self._config['compass'] = self.loadXML(9, 4)  # [open?,port,baudrate]
        self._config['Baro'] = self.loadXML(10, 4)  # [open?,port,baudrate]
        # [open?,port,safety distance,detected distance]
        self._config['lidar'] = self.loadXML(11, 4)
        # [Current Gear,Low Gear,Mid Gear,High Gear]
        self._config['gear'] = self.loadXML(12, 4)
        self._config['MoveTime'] = self.get_node(13, 1)
        self._config['BrakeTime'] = [-1] + self.loadXML(14, 3)
        self._config['DecideTime'] = self.loadXML(15, 2)
        self._config['LandingGear'] = self.loadXML(16, 3)
        self._config['IgnoreDegree'] = self.loadXML(17, 2)

    def loadXML(self, index, end, start=1):
        return [self.get_node(index, x) for x in range(start, end + 1)]

    def loadPIT(self):
        index = 5
        num = self.get_node(index, 1) - 1
        low = self.get_node(index, 2)
        mid = self.get_node(index, 3)
        hig = self.get_node(index, 4)
        var = hig - low
        return [num, low, mid, hig, var]

    def ch(self, index):
        num = self.get_node(index, 1) - 1
        low = self.get_node(index, 2)
        mid = self.get_node(index, 3)
        hig = self.get_node(index, 4)
        var = hig - low
        sign = self.get_node(index, 5)
        rate = self.get_node(index, 6)
        return [num, low, mid, hig, var, sign, rate]

    def isInt(self, x):
        try:
            return isinstance(int(x), int)
        except ValueError:
            return False

    def get_node(self, param1, param2):
        value = self._root[param1][param2].get('value')
        if self.isInt(value) is True:
            return int(value)
        else:
            return value

    def get_vehicle(self):
        return self._config['vehicle']

    def get_frame(self):
        return self._config['frame']

    def get_FC(self):
        return self._config['FC']

    def get_cloud(self):
        return self._config['cloud']

    def get_AIL(self):
        return self._config['AIL']

    def get_ELE(self):
        return self._config['ELE']

    def get_THR(self):
        return self._config['THR']

    def get_RUD(self):
        return self._config['RUD']

    def get_mode(self):
        return self._config['mode']

    def get_PIT(self):
        return self._config['PIT']

    def get_MCU(self):
        return self._config['MCU']

    def get_GPS(self):
        return self._config['GPS']

    def get_compass(self):
        return self._config['compass']

    def get_Baro(self):
        return self._config['Baro']

    def get_lidar(self):
        return self._config['lidar']

    def get_gear(self):
        return self._config['gear']

    def get_MD(self):
        return self._config['MoveTime']

    def get_BD(self):
        return self._config['BrakeTime']

    def get_DD(self):
        return self._config['DecideTime']

    def get_degree(self):
        return self._config['IgnoreDegree']

    def __str__(self):
        return json.dumps(self._config)

# Global config
config = Config()

if __name__ == "__main__":
    print config
