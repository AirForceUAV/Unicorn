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
        self._config['UAV'] = self._root.get('UAV')
        self.model = self._root.get('Model')
        self._config['Model'] = self.model
        self._config['MainController'] = self._root.get('MainController')
        self._config['AIL'] = self.channel(0)
        self._config['ELE'] = self.channel(1)
        self._config['THR'] = self.channel(2)
        self._config['RUD'] = self.channel(3)
        self._config['Mode'] = [self.get_node(
            4, 1) - 1, 0, self.get_node(4, 2)]
        if self.model == 'HELI':
            self._config['PIT'] = self.loadPIT(5)
        self._config['MCU'] = self.loadXML(6, 3)  # [open?,port,baudrate]
        self._config['GPS'] = self.loadXML(7, 3)  # [open?,port,baudrate]
        self._config['Compass'] = self.loadXML(8, 3)  # [open?,port,baudrate]
        self._config['Baro'] = self.loadXML(9, 3)  # [open?,bus,i2c]
        self._config['IMU'] = self.loadXML(10, 3)  # [open?,port,baudrate]

        # [open?,port,safety distance,detected distance]
        self._config['Lidar'] = self.loadXML(11, 4)
        self._config['Cloud'] = self.loadXML(12, 3)
        # [Current Gear,Low Gear,Mid Gear,High Gear]
        self._config['Gear'] = self.loadXML(13, 4)

    def loadXML(self, index, end, start=1):
        return [self.get_node(index, x) for x in range(start, end + 1)]

    def loadPIT(self, index):
        num = self.get_node(index, 1) - 1
        low = self.get_node(index, 2)
        mid = self.get_node(index, 3)
        hig = self.get_node(index, 4)
        var = hig - low
        return [num, low, mid, hig, var]

    def channel(self, index):
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
        if self.isInt(value):
            return int(value)
        else:
            return value

    def __str__(self):
        return json.dumps(self._config)

# Global config
config = Config()

if __name__ == "__main__":
    model = ['UAV', 'Model', 'MainController']
    _model = {}
    for m in model:
        _model[m] = config._config[m]
    print _model
    print config
