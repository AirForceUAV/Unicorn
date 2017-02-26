#! /usr/bin/env python
# -*- coding: utf-8 -*-

import threading
import time
from library import CutFrame2, Singleton, open_serial
from config import IMU
from tools import logger


class IMU(threading.Thread):
    __metaclass__ = Singleton

    def __init__(self, ORB):
        """
        Gyroscopemetre, Accelerometer, Magnetometer, Euler angles, Quaternions
        """
        super(IMU, self).__init__(name='IMU')
        self.FRAME_LEN = 52
        self.FRAME_HEAD = b'90ffa0'

        self.ACC_HEAD = b'a0'
        self.GYR_HEAD = b'b0'
        self.MAG_HEAD = b'c0'
        self.EUL_HEAD = b'd0'
        self.QUA_HEAD = b'd1'
        self.PRE_HEAD = b'f0'

        self.ACC_LEN = 7
        self.GYR_LEN = 7
        self.MAG_LEN = 7
        self.EUL_LEN = 7
        self.QUA_LEN = 17
        self.PRE_LEN = 5

        self.ACC_UNIT = 0.000244  # G
        self.GYR_UNIT = 0.061035  # °/s
        self.MAG_UNIT = 6         # Gauss
        self.EUL_UNIT = 100.0    # °

        self.ORB = ORB
        from config import IMU
        self._imu = open_serial(IMU, 115200)

    def run(self):
        logger.info('Initializing IMU....')
        while True:
            frame = self.RawFrame()
            Acc, Gyr, Mag, Eul, Qua = self.ParseIMU(frame)
            dic = {'IMU_State': True, 'ACC': Acc, 'GYR': Gyr,
                   'MAG': Mag, 'EUL': Eul, 'QUA': Qua}
            self.update(dic)

    def update(self, dictories):
        for (k, v) in dictories.items():
            self.ORB.publish(k, v)

    def _calculte_mantissa(self, bin_number, exponent):
        val = 1 if exponent > -127 else 0
        bit_count = -1
        bit_length = 0
        while bit_length <= 22:
            val += int(bin_number[bit_length]) * 2**bit_count
            bit_count -= 1
            bit_length += 1
        return val

    def convert_ieee754(self, hex_val):
        bin_pos = format(int(hex_val, 16), "0>32b")
        sign = (-1)**int(bin_pos[0], 2)
        _exponent = int(bin_pos[1:9], 2) - 127
        mantissa = self._calculte_mantissa(bin_pos[9:], _exponent)
        exponent = _exponent if _exponent > -127 else -126
        position = sign * 2**exponent * mantissa
        return position

    def convert_complement(self, hex_val):
        dec_val = int(hex_val, 16)
        if dec_val >= 2**15:
            dec_val = -(2**16 - dec_val)
        return dec_val

    def RawFrame(self):
        self._imu.flushInput()
        frame = self._imu.read(self.FRAME_LEN * 2).encode('hex')
        begin = frame.find(self.FRAME_HEAD)
        end = begin + self.FRAME_LEN * 2
        return frame[begin:end]

    def ParseIMU(self, package):
        a = package.find(self.ACC_HEAD)
        g = a + self.ACC_LEN * 2
        m = g + self.GYR_LEN * 2
        e = m + self.MAG_LEN * 2
        q = e + self.EUL_LEN * 2
        p = q + self.QUA_LEN * 2

        RawAcc = package[a + 2:a + self.ACC_LEN * 2]
        RawGyr = package[g + 2:g + self.GYR_LEN * 2]
        RawMag = package[m + 2:m + self.MAG_LEN * 2]
        RawEul = package[e + 2:e + self.EUL_LEN * 2]
        RawQua = package[q + 2:q + self.QUA_LEN * 2]
        RawPre = package[p + 2:p + self.PRE_LEN * 2]
        # print 'ACC:{},Gyr:{},MAG:{},EUL:{},QUA:{},Pre:{}'.format(RawAcc,
        # RawGyr, RawMag, RawEul, RawQua,RawPre)

        Acc = map(lambda x: round(x * self.ACC_UNIT, 4),
                  self.ParseFrag(RawAcc))
        Gyr = map(lambda x: round(x * self.GYR_UNIT, 4),
                  self.ParseFrag(RawGyr))
        Mag = map(lambda x: x * self.MAG_UNIT, self.ParseFrag(RawMag))
        Eul = map(lambda x: x / self.EUL_UNIT, self.ParseFrag(RawEul))
        Qua = map(lambda x: round(x, 4), self.ParseQua(RawQua))
        return Acc, Gyr, Mag, Eul, Qua

    def MergeFrame(self, package):
        return reduce(lambda x, y: x + y, package)

    def ReverseFrame(self, frame, length):
        return map(lambda x: self.MergeFrame(CutFrame2(x)[::-1]), CutFrame2(frame, length))

    def ParseQua(self, Qua):
        a = self.ReverseFrame(Qua, 8)
        return map(self.convert_ieee754, a)

    def ParseFrag(self, Frag):
        a = self.ReverseFrame(Frag, 4)
        return map(self.convert_complement, a)


if __name__ == "__main__":
    from library import Watcher
    from uORB import uORB
    ORB = uORB()
    imu = IMU(ORB)

    # a0='a0'+'edff7c003310'
    # b0='b0'+'000000000000'
    # c0='c0'+'f1ffedffdeff'
    # d0='d0'+'ebff50ff0800'
    # d1='d1'+'95dff43b6533ecba482a7d3c35fe7f3f'
    # f0='f0'+'00000000'
    # frame = '90ff'+a0+b0+c0+d0+d1+f0
    # Acc,Gyr, Mag, Eul, Qua=imu.ParseIMU(frame)
    # print Acc, Gyr, Mag, Eul, Qua

    Watcher()
    imu.start()
    while not ORB.subscribe('IMU_State'):
        time.sleep(.1)
    while True:
        Acc = ORB.subscribe('ACC')
        Gyr = ORB.subscribe('GYR')
        Mag = ORB.subscribe('MAG')
        Eul = ORB.subscribe('EUL')
        Qua = ORB.subscribe('QUA')
        print 'ACC:{}, GYR:{}, MAG:{}, EUL:{}, QUA:{}'.format(Acc, Gyr, Mag, Eul, Qua)
        raw_input('Next')
