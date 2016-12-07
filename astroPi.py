#!/usr/bin/evn python
# coding:utf-8

from sense_hat import SenseHat
from library import Singleton
import time

"""
The Sense HAT has the following features:
1 Gyroscope, accelerometer, and magnetometer sensor
2 Temperature and humidity sensor
3 Barometric pressure sensor
4 8×8 RGB LED display
5 Mini joystick
"""


class AstroPi(object):
    __metaclass__ = Singleton

    def __init__(self):
        self.sense = SenseHat()
        self.sense.set_imu_config(True, True, True)

    def get_pressure(self):
        pressure = self.sense.get_pressure()
        if pressure is 0:
            time.sleep(.5)
            return self.get_pressure()
        else:
            return pressure

    def get_alt(self):
        mbar = self.get_pressure() / 1013.25
        return round((1 - mbar**0.190284) * 145366.45 * 0.3048, 2)

    def get_heading(self):
        north = self.sense.get_compass()
        offset = 0
        return int((north + offset) % 360)

    def get_radians(self):
        return self.sense.get_orientation_radians()

    def get_degrees(self):
        return self.sense.get_orientation_degrees()

    def get_orientation(self):
        return self.sense.get_orientation()

    def get_compass_raw(self):
        return self.sense.get_compass_raw()

    def get_gyroscope(self):
        return self.sense.get_gyroscope()

    def get_gyroscope_raw(self):
        return self.sense.get_gyroscope_raw()

    def get_accelerometer(self):
        return self.sense.get_accelerometer()

    def get_accelerometer_raw(self):
        return self.sense.get_accelerometer_raw()

    def printInfo(self):
        print "Pressure", self.get_pressure()
        print "Altitude", self.get_alt()
        print 'Heading', self.get_heading()
        print "Radians", self.get_radians()
        print 'Degree', self.get_degrees()
        print "Orientation", self.get_orientation()
        print "compass", self.get_compass_raw()
        print "gyroscope", self.get_gyroscope()
        print "acceler", self.get_accelerometer()

sense = AstroPi()

if __name__ == "__main__":
    # while True:
    #     sense.sense.clear()
    #     sense.printInfo()
    #     raw_input("Next")
    from sense_hat import SenseHat

    sense = SenseHat()
    sense.clear()

    edge = [0, 1, 2, 3, 4, 5, 6, 7, 15, 23, 31, 39, 47, 55,
            63, 62, 61, 60, 59, 58, 57, 56, 48, 40, 32, 24, 16, 8]
    length = len(edge)
    ratio = length / 360.0

    while True:
        o = sense.get_orientation()
        pitch = o["pitch"]
        roll = o["roll"]
        yaw = o["yaw"]

        yaw_list_position = int(yaw * ratio)

        yaw_pixel_number = edge[yaw_list_position]

        y = yaw_pixel_number // 8
        x = yaw_pixel_number % 8

        sense.set_pixel(x, y, 255, 255, 255)