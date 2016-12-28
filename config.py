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
        self._config['Mode'] = self.loadFUN(4)
        if self.model == 'HELI':
            self._config['Rate'] = self.loadFUN(5)
            self._config['PIT'] = self.loadPIT(6)
        else:
            self._config['Aux1'] = self.loadFUN(5)
            self._config['Aux2'] = self.loadFUN(6)
        self._config['Switch'] = self.loadFUN(7)
        # [Current Gear,Low Gear,Mid Gear,High Gear]
        self._config['Gear'] = self.loadHAL(8, 4)

    def loadFUN(self, index):
        return [self.node(index, 1) - 1, 0, self.node(index, 2)]

    def loadHAL(self, index, end, start=1):
        return [self.node(index, x) for x in range(start, end + 1)]

    def loadPIT(self, index):
        num = self.node(index, 1) - 1
        low = self.node(index, 2)
        mid = self.node(index, 3)
        hig = self.node(index, 4)
        var = hig - low
        return [num, low, mid, hig, var]

    def channel(self, index):
        num = self.node(index, 1) - 1
        low = self.node(index, 2)
        mid = self.node(index, 3)
        hig = self.node(index, 4)
        var = hig - low
        sign = self.node(index, 5)
        rate = self.node(index, 6)
        return [num, low, mid, hig, var, sign, rate]

    def isInt(self, x):
        try:
            return isinstance(int(x), int)
        except ValueError:
            return False

    def node(self, param1, param2):
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
    print config
