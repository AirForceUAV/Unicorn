#!/usr/bin/evn python
# coding:utf-8

import sys
sys.path.append('..')
import os
import numpy
import toml
from lib.config import config

exit = False
UAV = config.drone['UAV']
T2P_file = os.path.join('..', 'ML', UAV + ".t2p")
ratio_file = os.path.join('..', 'ML', UAV + ".ratio")


class fitting:

    def __init__(self, X, Y):
        self.x = numpy.array(X)
        self.y = numpy.array(Y)

    def fitting(self, n):
        # 系数
        self.z = numpy.polyfit(self.x, self.y, n)
        # 拟合函数
        self.p = numpy.poly1d(self.z)
        # 误差
        self.error = numpy.abs(self.y - numpy.polyval(self.p, self.x))
        # 最小平方误差
        self.ER2 = numpy.sum(numpy.power(self.error, 2)) / len(self.x)
        return self.z, self.p

    def geterror(self):
        return self.error, self.ER2

    def show(self):
        from matplotlib import pyplot as pl
        figure1 = pl.figure()
        pl.plot(self.x, self.y, 'ro-', markersize=7,
                figure=figure1, label='origin data')
        pl.plot(self.x, numpy.polyval(self.p, self.x),
                markersize=7, figure=figure1, label='fitting data')
        pl.legend()
        pl.show()

    def predict(self, x):
        return numpy.polyval(self.p, x)


def change_exit():
    global exit
    exit = True


def collect_pwm(ORB):
    import time
    # import keyboard
    # keyboard.add_hotkey('esc', change_exit)
    global exit
    global T2P_file
    raw_input('Start collecting pwm --> [enter]:start [esc]:exit')

    THR_PIT = {}
    times = 0
    while times < 2000:
        input = ORB.subscribe('ChannelsInput')
        # print input
        THR_PIT[str(input[2])] = input[5]
        times+=1
        time.sleep(.01)
        
    exit = False

    message = toml.dumps(THR_PIT)
    # print message
    with open(T2P_file, 'w') as f:
        f.write(message)

    print('End collecting')


def generate_ratio():
    global T2P_file
    global ratio_file
    with open(T2P_file, 'r') as f:
        THR_PIT = toml.loads(f.read())
    X1 = []
    Y1 = []
    X2 = []
    Y2 = []
    X3 = []
    Y3 = []
    for THR, PIT in THR_PIT.iteritems():
        THR = int(THR)
        if THR < 1005:
            X1.append(THR)
            Y1.append(PIT)
        elif THR >= 1005 and THR < 1400:
            X2.append(THR)
            Y2.append(PIT)
        else:
            X3.append(THR)
            Y3.append(PIT)

    F1 = fitting(X1, Y1)
    z1, p1 = F1.fitting(3)

    F2 = fitting(X2, Y2)
    z2, p2 = F2.fitting(3)

    F3 = fitting(X3, Y3)
    z3, p3 = F3.fitting(3)

    T2P = {}
    T2P = {k: v.tolist()
           for k, v in zip(['z1', 'z2', 'z3'], [z1, z2, z3])}

    with open(ratio_file, 'w') as f:
        message = toml.dumps(T2P)
        print message
        f.write(message)

    # F1.show()
    # F2.show()
    # F3.show()


def check_error():
    global T2P_file
    with open(T2P_file, 'r') as f:
        THR_PIT = toml.loads(f.read())
    # with open(file)
    for THR, PIT in THR_PIT.iteritems():
        p = THR2PIT(int(THR))
        print THR, PIT, p, PIT - p


def THR2PIT(x):
    global ratio_file
    with open(ratio_file, 'r') as f:
        ratios = toml.loads(f.read())
    if x < 1005:
        z = ratios['z1']
    elif x >= 1005 and x < 1400:
        z = ratios['z2']
    else:
        z = ratios['z3']

    coefficient = numpy.array(z)
    # Fitting Function
    fitfunction = numpy.poly1d(coefficient)
    return int(numpy.polyval(fitfunction, x))

if __name__ == '__main__':

    from lib.tools import Watcher
    from AF_uORB.uORB import uORB
    from AF_Sbus.receiver import sbus_receive_start

    ORB = uORB()
    Watcher()

    sbus_receive_start(ORB)

    print('Sbus is OK')

    collect_pwm(ORB)

    print('Generate Ratio ...')
    generate_ratio()
    print("Check Error ...")
    check_error()

    # print THR2PIT(1000)
