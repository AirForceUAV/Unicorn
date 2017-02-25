#!/usr/bin/evn python
# coding:utf-8

import numpy


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


def THR2PIT(x):
    if x < 1005:
        z = [-2.33768242e-07,  3.30021519e-04, -6.02869587e-01, 1.29651932e+03]
    elif x >= 1005 and x < 1400:
        z = [1.14590757e-07, -4.24145284e-04, -8.53510386e-02, 1.18774116e+03]
    else:
        z = [1.09535756e-06, -5.23323098e-03, 7.57707790e+00, -2.80403030e+03]
    coefficient = numpy.array(z)
    # Fitting Function
    fitfunction = numpy.poly1d(coefficient)
    return int(numpy.polyval(fitfunction, x))

if __name__ == '__main__':

    with open('PitchCurve.ML', 'r') as f:
        lines = f.readlines()

    # X1 = []
    # Y1 = []
    # X2 = []
    # Y2 = []
    # X3 = []
    # Y3 = []
    # for line in lines:
    #     line = line.split(',')
    #     line = map(int, line)
    #     if line[0] < 1005:
    #         X1.append(line[0])
    #         Y1.append(line[1])
    #     elif line[0] >= 1005 and line[0] < 1400:
    #         X2.append(line[0])
    #         Y2.append(line[1])
    #     else:
    #         X3.append(line[0])
    #         Y3.append(line[1])

    # F1 = fitting(X1, Y1)
    # z1, p1 = F1.fitting(3)

    # F2 = fitting(X2, Y2)
    # z2, p2 = F2.fitting(3)

    # F3 = fitting(X3, Y3)
    # z3, p3 = F3.fitting(3)

    # print 'poly1：'
    # print z1
    # print 'poly2：'
    # print z2
    # print 'poly3：'
    # print z3

    # F1.show()
    # F2.show()
    # F3.show()

    # for line in lines:
    #     line = line.split(',')
    #     line = map(int, line)
    #     if line[0] < 1005:
    #         pitch = int(F1.predict(line[0]))
    #     elif line[0] >= 1005 and line[0] < 1400:
    #         pitch = int(F2.predict(line[0]))
    #     else:
    #         pitch = int(F3.predict(line[0]))
    #     print line[0], pitch, line[1], pitch - line[1]

    for line in lines:
        line = line.split(',')
        line = map(int, line)
        p = THR2PIT(line[0])
        print line[0], line[1], p, line[1] - p
