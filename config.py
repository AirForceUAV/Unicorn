#!/usr/bin/evn python
# coding:utf-8

from library import Singleton, element


class Config(object):
    __metaclass__ = Singleton

    def __init__(self):
        file_name = 'Vehicle.xml'
        root_node = element(file_name)
        ID = int(root_node.get('ID'))
        self._root = root_node[ID]
        self._vehicle = self._root.get(
            'vehicle')                 # vehicle type
        self._frame = self._root.get('frame')
        self._FC = self._root.get('FlightController')     # FC  version
        self._cloud = [self.get_node(0, 1), self.get_node(
            0, 2), self.get_node(0, 3)]  # [open?,ip,port]
        self._AIL = self.ch(1)
        self._ELE = self.ch(2)
        self._THR = self.ch(3)
        self._RUD = self.ch(4)
        self._PIT = [self.get_node(
            5, 1) - 1, self.get_node(5, 2)]
        # [ch No.,mode pwm]
        self._mode = [self.get_node(6, 1) - 1, self.get_node(6, 2)]
        self._MCU = [self.get_node(7, 1), self.get_node(7, 2), self.get_node(
            7, 3), self.get_node(7, 4)]  # [open?,port,baudrate]
        self._GPS = [self.get_node(8, 1), self.get_node(8, 2), self.get_node(
            8, 3), self.get_node(8, 4)]  # [open?,port,baudrate]
        self._Comp = [self.get_node(9, 1), self.get_node(9, 2), self.get_node(
            9, 3), self.get_node(9, 4)]  # [open?,port,baudrate]
        self._Baro = [
            self.get_node(
                10, 1), self.get_node(
                10, 2), self.get_node(
                10, 3), self.get_node(
                    10, 4)]  # [open?,port,baudrate]
        self._lidar = [
            self.get_node(
                11, 1), self.get_node(
                11, 2), self.get_node(
                11, 3), self.get_node(
                    11, 4)]  # [open?,port,safety distance,detected distance]
        self._gear = [
            self.get_node(
                12, 1), self.get_node(
                12, 2), self.get_node(
                12, 3), self.get_node(
                    12, 4)]  # [Current Gear,Low Gear,Mid Gear,High Gear]
        self._MD = [self.get_node(13, 1)]
        self._BD = [1, self.get_node(14, 1), self.get_node(
            14, 2), self.get_node(14, 3)]
        self._DD = [self.get_node(15, 1), self.get_node(15, 2)]
        self._LG = [self.get_node(16, 1), self.get_node(
            16, 2), self.get_node(16, 3)]
        self._degree = [self.get_node(17, 1), self.get_node(17, 2)]

    # [ch No.,low PWM,mid PWM,high PWM,variation,sign]
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
        return self._vehicle

    def get_frame(self):
        return self._frame

    def get_FC(self):
        return self._FC

    def get_cloud(self):
        return self._cloud

    def get_AIL(self):
        return self._AIL

    def get_ELE(self):
        return self._ELE

    def get_THR(self):
        return self._THR

    def get_RUD(self):
        return self._RUD

    def get_mode(self):
        return self._mode

    def get_PIT(self):
        return self._PIT

    def get_MCU(self):
        return self._MCU

    def get_GPS(self):
        return self._GPS

    def get_compass(self):
        return self._Comp

    def get_Baro(self):
        return self._Baro

    def get_lidar(self):
        return self._lidar

    def get_gear(self):
        return self._gear

    def get_MD(self):
        return self._MD

    def get_BD(self):
        return self._BD

    def get_DD(self):
        return self._DD

    def get_degree(self):
        return self._degree

# Global config
config = Config()

if __name__ == "__main__":
    print 'Vehicle frame:{}'.format(config.get_frame())
    print 'Vehicle type:{}'.format(config.get_vehicle())
    print 'FC firmware:{}'.format(config._FC)
    print 'Cloud:', config.get_cloud()
    print 'AIL:', config.get_AIL()
    print 'ELE:', config.get_ELE()
    print 'THR:', config.get_THR()
    print 'RUD:', config.get_RUD()
    print 'PIT:', config.get_PIT()
    print 'Mode:', config.get_mode()
    print 'MCU:', config.get_MCU()
    print 'GPS:', config.get_GPS()
    print 'Compass:', config.get_compass()
    print 'Barometre:', config.get_Baro()
    print 'Lidar:', config.get_lidar()
    print 'Gear:', config.get_gear()
    print 'Movement Duration', config.get_MD()
    print 'Brake Duration', config.get_BD()
    print 'Decision Duration', config.get_DD()
    print 'Ignore Degree', config.get_degree()
